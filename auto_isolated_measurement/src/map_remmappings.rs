use std::collections::HashMap;

use strsim::levenshtein;

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

    (candidates[min_index.unwrap()].clone(), min_average_distance)
}

fn get_core_str(topic: &str) -> String {
    topic
        .replace('"', "")
        .split('/')
        .collect::<Vec<&str>>()
        .last()
        .unwrap()
        .to_string()
}

pub fn map_remappings(
    mut original_remappings: Vec<(String, String)>,
    mut subs: Vec<String>,
    mut pubs: Vec<String>,
) -> Option<HashMap<String, String>> {
    original_remappings.retain(|(from, to)| !from.contains("debug") || !to.contains("debug"));

    if original_remappings.is_empty() {
        return None;
    }

    let mut fixed_remappings = HashMap::new();

    // First, the topics for which the remapping string is directly specified are mapped.
    original_remappings.retain(|(from, to)| {
        if to.starts_with('\"') {
            fixed_remappings.insert(from.replace('\"', ""), to.replace('\"', ""));
            subs.retain(|sub_| sub_ != to);
            pubs.retain(|pub_| pub_ != to);
            false
        } else {
            true
        }
    });

    if original_remappings.is_empty() {
        return Some(fixed_remappings);
    }

    // If there is one input and one output, they correspond to sub and pub respectively.
    // TODO: refactor
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

    original_remappings.retain(|(from, to)| {
        if from.contains("input") || to.contains("input") {
            let core_str = get_core_str(from);
            let count = subs.iter().filter(|sub_| sub_.contains(&core_str)).count();
            if count == 1 {
                let sub_topic_name = subs.remove(
                    subs.iter()
                        .position(|sub_| sub_.contains(&core_str))
                        .unwrap(),
                );
                fixed_remappings.insert(from.trim_matches('\"').to_string(), sub_topic_name);
                false
            } else {
                true
            }
        } else if from.contains("output") || to.contains("output") {
            let core_str = get_core_str(from);
            let count = pubs.iter().filter(|pub_| pub_.contains(&core_str)).count();
            if count == 1 {
                let pub_topic_name = pubs.remove(
                    pubs.iter()
                        .position(|pub_| pub_.contains(&core_str))
                        .unwrap(),
                );
                fixed_remappings.insert(from.trim_matches('\"').to_string(), pub_topic_name);
                false
            } else {
                true
            }
        } else {
            let core_str = get_core_str(from);
            let count = subs.iter().filter(|sub_| sub_.contains(&core_str)).count()
                + pubs.iter().filter(|pub_| pub_.contains(&core_str)).count();
            if count == 1 {
                let topic_name = if !subs.is_empty() {
                    subs.remove(
                        subs.iter()
                            .position(|sub_| sub_.contains(&core_str))
                            .unwrap(),
                    )
                } else {
                    pubs.remove(
                        pubs.iter()
                            .position(|pub_| pub_.contains(&core_str))
                            .unwrap(),
                    )
                };
                fixed_remappings.insert(from.trim_matches('\"').to_string(), topic_name);
                false
            } else {
                true
            }
        }
    });

    if original_remappings.is_empty() {
        return Some(fixed_remappings);
    }

    while (!subs.is_empty() || !pubs.is_empty()) && !original_remappings.is_empty() {
        let mut min_distances: Vec<usize> = Vec::with_capacity(original_remappings.len());
        let mut closest_strs: Vec<String> = Vec::with_capacity(original_remappings.len());
        for (from, to) in original_remappings.iter() {
            if from.contains("input") || to.contains("input") {
                let (closest_str, min_distance) = get_closest_str(to, from, &subs);
                closest_strs.push(closest_str);
                min_distances.push(min_distance);
            } else if from.contains("output") || to.contains("output") {
                let (closest_str, min_distance) = get_closest_str(to, from, &pubs);
                closest_strs.push(closest_str);
                min_distances.push(min_distance);
            } else {
                unreachable!();
            }
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
