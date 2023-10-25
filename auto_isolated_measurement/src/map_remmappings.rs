use std::collections::HashMap;

use strsim::levenshtein;

// fn choice_closest_str(
//     first_target_str: &str,
//     second_target_str: &str,
//     second_threshold: usize,
//     candidates: &mut Vec<String>,
// ) -> Option<String> {
//     let distances: Vec<_> = candidates
//         .iter()
//         .map(|candidate| levenshtein(first_target_str, candidate))
//         .collect();
//     let (first_most_closest_i, &most_closest_distance) = distances
//         .iter()
//         .enumerate()
//         .min_by_key(|&(_, distance)| distance)?;
//     let close_candidates_indices: Vec<_> = distances
//         .iter()
//         .enumerate()
//         .filter(|&(index, &distance)| {
//             index != first_most_closest_i && distance <= most_closest_distance + second_threshold
//         })
//         .map(|(index, _)| index)
//         .collect();

//     let closest_i = close_candidates_indices
//         .into_iter()
//         .min_by_key(|&index| levenshtein(second_target_str, &candidates[index]))
//         .unwrap_or(first_most_closest_i);

//     Some(candidates.remove(closest_i))
// }

fn choice_closest_str(
    first_target_str: &str,
    second_target_str: &str,
    candidates: &mut Vec<String>,
) -> Option<String> {
    let mut min_index = None;
    let mut min_average_distance = usize::MAX;

    for (index, candidate) in candidates.iter().enumerate() {
        let distance =
            levenshtein(first_target_str, candidate) + levenshtein(second_target_str, candidate);
        if distance < min_average_distance {
            min_index = Some(index);
            min_average_distance = distance;
        }
    }

    min_index.map(|index| candidates.remove(index))
}

pub fn map_remappings(
    mut original_remappings: Vec<(String, String)>,
    mut subs: Vec<String>,
    mut pubs: Vec<String>,
) -> Option<HashMap<String, String>> {
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

    // If there is one input and one output, they correspond to sub and pub respectively.
    // TODO: refactor
    if original_remappings.len() <= 2 && subs.len() <= 1 && pubs.len() <= 1 {
        original_remappings.retain(|(from, _)| {
            if from.contains("input") {
                fixed_remappings.insert(from.replace('\"', ""), subs.remove(0));
                false
            } else if from.contains("output") {
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
        })
    }

    original_remappings.retain(|(from, to)| {
        if from.contains("input") {
            fixed_remappings.insert(
                from.replace('\"', ""),
                choice_closest_str(to, from, &mut subs).unwrap(),
            );
            false
        } else if from.contains("output") {
            fixed_remappings.insert(
                from.replace('\"', ""),
                choice_closest_str(to, from, &mut pubs).unwrap(),
            );
            false
        } else {
            unreachable!()
        }
    });

    if fixed_remappings.is_empty() {
        None
    } else {
        Some(fixed_remappings)
    }
}
