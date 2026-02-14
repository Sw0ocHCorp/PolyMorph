use std::{fs::{File, create_dir}, io::{self, Write}};

use chrono::Local;

const LOGS_FILE_NAME: &str= "logs";

pub struct FileLogger {
    folder_path: String,
    file_name: String,
    file: Option<File>,
}

impl FileLogger {
    pub fn new(folder_path: String) -> Self {
        match (create_dir(&folder_path)) {
            Ok(_) => println!("Create Directory"),
            Err(e) if e.kind() == io::ErrorKind::AlreadyExists=> println!("Directory already Exist"), 
            Err(e) => println!("Failed to create directory: {}", e)
        }
        let now = Local::now().format("%Y-%m-%d_%H-%M-%S%.f").to_string();
;
        /*if let Some(idx)= now.find(" +") {
            let _= now.split_off(idx);
            now= now.replace(".", ",").replace(" ", "|").replace(":", "-").replace("-", "_");
        }*/
        let file_name= LOGS_FILE_NAME.to_string() + "_" + &now + ".txt";
        let file_path= folder_path.clone()+ "/" + &file_name.clone();
        match File::create(&file_path) {
            Ok(file) => {
                return Self { folder_path: folder_path, file_name:  file_name.clone(), file: Some(file)};
            },
            Err(e) => {
                println!("{} for {}", e.kind(), file_name);
                return Self { folder_path: folder_path, file_name:  file_name, file: None};
            },
        }
        
    }

    pub fn add_logs(&self, content: String) {
        if let Some(mut file) = self.file.as_ref() {
            if let Err(_)= file.write(content.as_bytes()) {
                println!("Error writing logs");
            }
        }
    }
}