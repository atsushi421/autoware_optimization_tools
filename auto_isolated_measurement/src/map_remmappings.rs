use std::collections::HashMap;

use strsim::levenshtein;

fn choise_closest_str(target_str: &str, candidates: &mut Vec<String>) -> Option<String> {
    let closest_index = candidates
        .iter()
        .enumerate()
        .min_by_key(|&(_, candidate)| levenshtein(target_str, candidate))
        .map(|(index, _)| index);

    closest_index.map(|index| candidates.remove(index).to_string())
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
                choise_closest_str(to, &mut subs).unwrap(),
            );
            false
        } else if from.contains("output") {
            fixed_remappings.insert(
                from.replace('\"', ""),
                choise_closest_str(to, &mut pubs).unwrap(),
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
