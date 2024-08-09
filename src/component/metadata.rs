use regex::Regex;
use serde::{Deserialize, Serialize};
use serde_yaml::Value;

#[derive(Debug, Deserialize, Serialize, Default)]
pub struct Metadata {
    pub layout: Option<String>,
    pub title: Option<String>,
    pub date: Option<String>,
    pub excerpt: Option<String>,
    pub tags: Vec<String>,
}

pub fn extract_yaml_block_and_rest(text: &str) -> (Option<&str>, Option<&str>) {
    let re = Regex::new(r"(?s)^---\n(.*?)\n---\n(.*)").unwrap();
    if let Some(caps) = re.captures(text) {
        let yaml_block = caps.get(1).map(|m| m.as_str());
        let rest_of_text = caps.get(2).map(|m| m.as_str());
        (yaml_block, rest_of_text)
    } else {
        (None, None)
    }
}

pub fn parse_yaml(yaml_block: &str) -> Result<Metadata, serde_yaml::Error> {
    serde_yaml::from_str(yaml_block)
}
