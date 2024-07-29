use serde_json::to_string;
use std::fs::{self, File};
use std::io::Write;
use std::path::Path;

fn main() {
    update_post();
}

fn update_post() {
    let markdown_dir = Path::new("posts/");
    let output_file = Path::new("posts/list.json");

    let mut file_list = vec![];

    if markdown_dir.is_dir() {
        for entry in fs::read_dir(markdown_dir).unwrap() {
            let entry = entry.unwrap();
            let path = entry.path();
            if path.is_file() {
                if let Some(extension) = path.extension() {
                    if extension == "md" {
                        if let Some(file_name) = path.file_name() {
                            file_list.push(file_name.to_string_lossy().into_owned());
                        }
                    }
                }
            }
        }
    }

    let json_data = serde_json::to_string(&file_list).unwrap();
    let mut file = File::create(output_file).unwrap();
    file.write_all(json_data.as_bytes()).unwrap();
}
