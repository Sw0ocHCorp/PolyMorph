use std::{fs::{File, create_dir}, io};

use chrono::Local;

const LOGS_FILE_NAME: &str= "logs";

pub struct FileLogger {
    folder_path: String,
    file_name: String,
    file: Option<File>,
}

impl FileLogger {
    fn new(folder_path: String) -> Self {
        match (create_dir(&folder_path)) {
            Ok(_) => println!("Create Directory"),
            Err(e) if e.kind() == io::ErrorKind::AlreadyExists=> println!("Directory already Exist"), 
            Err(e) => println!("Failed to create directory: {}", e)
        }
        let file_name= LOGS_FILE_NAME.to_string() + &Local::now().to_string();
        match File::create(file_name) {
            Ok(file) => {
                return Self { folder_path: folder_path, file_name:  LOGS_FILE_NAME.to_string() + &Local::now().to_string(), file: Some(file)};
            },
            Err(_) => {
                return Self { folder_path: folder_path, file_name:  LOGS_FILE_NAME.to_string() + &Local::now().to_string(), file: None};
            },
        }
        
    }
}