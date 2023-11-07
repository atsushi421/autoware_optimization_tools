use std::collections::HashMap;

use strsim::levenshtein;

use crate::utils::remove_by_str;

fn get_closest_str(target_str0: &str, target_str1: &str, candidates: &[String]) -> (String, usize) {
    let mut min_index = None;
    let mut min_average_distance = usize::MAX;

    for (index, candidate) in candidates.iter().enumerate() {
        let ave_distance =
            levenshtein(target_str0, candidate) + levenshtein(target_str1, candidate) / 2;
        if ave_distance < min_average_distance {
            min_index = Some(index);
            min_average_distance = ave_distance;
        }
    }

    if candidates.is_empty() {
        unreachable!("candidates is empty");
    }

    (candidates[min_index.unwrap()].clone(), min_average_distance)
}

fn get_core_str(original_str: &str) -> String {
    let core_str = original_str
        .replace('"', "")
        .split('/')
        .collect::<Vec<&str>>()
        .last()
        .unwrap()
        .to_string();
    if core_str.contains("odom") {
        "kinematic_state".to_string()
    } else if core_str.contains("object1") {
        "clustering".to_string() // HACK: for object_association_meager1
    } else if core_str.contains("reference_trajectory") {
        "obstacle_avoidance_planner".to_string() // HACK: for planning_evaluator
    } else if core_str.contains("velocity_limit_clear_command_from_internal") {
        "clear_velocity_limit".to_string() // HACK: for external_velocity_limit_selector
    } else if core_str.contains("ekf_trigger_node") {
        "/initialpose3d".to_string() // HACK: for pose_initializer
    } else {
        core_str
    }
}

pub fn fix_remappings(
    original_remappings: &mut Vec<(String, String)>,
    namespace: &str,
    node_name: &str,
    subs: &mut Vec<String>,
    pubs: &mut Vec<String>,
) -> Option<HashMap<String, String>> {
    original_remappings.retain(|(from, to)| {
        !(from.contains("debug")
            || to.contains("debug")
            || from.contains("service")
            || to.contains("service")
            || from.contains("client")
            || to.contains("client")
            || from.contains("srv")
            || to.contains("srv"))
    });

    if original_remappings.is_empty() {
        return None;
    }

    // Assign the omitted namespace and nodename
    let mut converted_remappings = Vec::new();
    let omitted_str = format!("{}/{}", namespace, node_name);
    original_remappings.retain(|(from, to)| {
        let from_fixed = if from.starts_with("\"~") {
            Some(from.replace('~', &omitted_str))
        } else {
            None
        };
        let to_fixed = if to.starts_with("\"~") {
            Some(to.replace('~', &omitted_str))
        } else if to.starts_with('\"') && !to.starts_with("\"/") {
            Some(format!("\"{}/{}", namespace, to.trim_start_matches('"'))) // Only namespace is assigned to this description method.
        } else {
            None
        };

        if from_fixed.is_some() || to_fixed.is_some() {
            converted_remappings.push((
                from_fixed.unwrap_or(from.to_string()),
                to_fixed.unwrap_or(to.to_string()),
            ));
            false
        } else {
            true
        }
    });
    original_remappings.extend(converted_remappings);

    let mut fixed_remappings = HashMap::new();

    // First, the topics for which the remapping string is directly specified are mapped.
    original_remappings.retain(|(from, to)| {
        if to.starts_with('\"') {
            let from_fixed = from.trim_matches('\"');
            let to_fixed = to.trim_matches('\"');
            fixed_remappings.insert(from_fixed.to_string(), to_fixed.to_string());
            subs.retain(|sub_| sub_ != to_fixed);
            pubs.retain(|pub_| pub_ != to_fixed);
            false
        } else {
            true
        }
    });

    if original_remappings.is_empty() {
        return Some(fixed_remappings);
    }

    // If there is one input and one output, they correspond to sub and pub respectively.
    if original_remappings.len() <= 2 && subs.len() <= 1 && pubs.len() <= 1 {
        original_remappings.retain(|(from, to)| {
            if from.contains("input") || to.contains("input") {
                fixed_remappings.insert(from.replace('\"', ""), subs.remove(0));
                false
            } else if from.contains("output") || to.contains("output") {
                fixed_remappings.insert(from.replace('\"', ""), pubs.remove(0));
                false
            } else if subs.len() + pubs.len() == 1 {
                fixed_remappings.insert(
                    from.replace('\"', ""),
                    if subs.is_empty() {
                        pubs.remove(0)
                    } else {
                        subs.remove(0)
                    },
                );
                false
            } else {
                unreachable!()
            }
        });

        return Some(fixed_remappings);
    }

    // Fix by core_str
    original_remappings.retain(|(from, to)| {
        let core_str = get_core_str(from);

        if from.contains("input") || to.contains("input") {
            if subs.iter().filter(|sub_| sub_.contains(&core_str)).count() == 1 {
                fixed_remappings.insert(
                    from.trim_matches('\"').to_string(),
                    remove_by_str(subs, &core_str),
                );
                false
            } else {
                true
            }
        } else if from.contains("output") || to.contains("output") {
            if pubs.iter().filter(|pub_| pub_.contains(&core_str)).count() == 1 {
                fixed_remappings.insert(
                    from.trim_matches('\"').to_string(),
                    remove_by_str(pubs, &core_str),
                );
                false
            } else {
                true
            }
        } else if subs.iter().filter(|sub_| sub_.contains(&core_str)).count()
            + pubs.iter().filter(|pub_| pub_.contains(&core_str)).count()
            == 1
        {
            let combined = [&subs[..], &pubs[..]].concat();
            let topic_name = match combined.iter().position(|x| x.contains(&core_str)) {
                Some(pos) => {
                    if pos < subs.len() {
                        subs.remove(pos)
                    } else {
                        pubs.remove(pos - subs.len())
                    }
                }
                None => unreachable!(),
            };
            fixed_remappings.insert(from.trim_matches('\"').to_string(), topic_name);
            false
        } else {
            true
        }
    });

    if original_remappings.is_empty() {
        return Some(fixed_remappings);
    }

    // Fix by levenshtein distance
    while (!subs.is_empty() || !pubs.is_empty()) && !original_remappings.is_empty() {
        let mut min_distances: Vec<usize> = Vec::with_capacity(original_remappings.len());
        let mut closest_strs: Vec<String> = Vec::with_capacity(original_remappings.len());

        original_remappings.retain(|(from, to)| {
            if from.contains("input") || to.contains("input") {
                if subs.is_empty() {
                    return false; // remove
                }

                let (closest_str, min_distance) = get_closest_str(to, from, &subs);
                closest_strs.push(closest_str);
                min_distances.push(min_distance);
            } else if from.contains("output") || to.contains("output") {
                if pubs.is_empty() {
                    return false; // remove
                }

                let (closest_str, min_distance) = get_closest_str(to, from, &pubs);
                closest_strs.push(closest_str);
                min_distances.push(min_distance);
            } else {
                let combined = [&subs[..], &pubs[..]].concat();
                let (closest_str, min_distance) = get_closest_str(to, from, &combined);
                closest_strs.push(closest_str);
                min_distances.push(min_distance);
            }
            true
        });

        if original_remappings.is_empty() {
            break;
        }

        let min_index = min_distances
            .iter()
            .enumerate()
            .min_by_key(|&(_, v)| v)
            .map(|(i, _)| i)
            .unwrap();

        let (from, _) = original_remappings.remove(min_index);
        let closest_str = closest_strs.remove(min_index);
        subs.retain(|sub_| sub_ != &closest_str);
        pubs.retain(|pub_| pub_ != &closest_str);
        fixed_remappings.insert(from.replace('\"', ""), closest_str);
    }

    if fixed_remappings.is_empty() {
        None
    } else {
        Some(fixed_remappings)
    }
}
