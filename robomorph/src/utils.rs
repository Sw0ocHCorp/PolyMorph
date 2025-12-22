pub fn contain_bytes(src_bytes: Vec<u8>, target_bytes: Vec<u8>) -> i128{
    let mut start_index= -1;
    if src_bytes.len() >= target_bytes.len() {
        for i in 0..src_bytes.len() - (target_bytes.len() - 1) {
            if src_bytes[i] == target_bytes[0] {
                start_index= i as i128;
                for j in 0..target_bytes.len() {
                    if src_bytes[i+j] != target_bytes[j] {
                        start_index= -1;
                    }
                }
            }
        }
    }
    return start_index;
}