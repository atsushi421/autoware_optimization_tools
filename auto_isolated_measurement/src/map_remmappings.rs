use std::collections::HashMap;

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
    if original_remappings.len() <= 2 && subs.len() <= 1 && pubs.len() <= 1 {
        for (from, _) in original_remappings {
            if from.contains("input") {
                fixed_remappings.insert(from.replace('\"', ""), subs[0].clone());
            } else if from.contains("output") {
                fixed_remappings.insert(from.replace('\"', ""), pubs[0].clone());
            }
        }
    }

    // TODO: support more cases

    if fixed_remappings.is_empty() {
        None
    } else {
        Some(fixed_remappings)
    }
}
