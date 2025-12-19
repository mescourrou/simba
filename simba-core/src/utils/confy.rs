use confy::ConfyError;


pub fn detailed_error(error: &ConfyError) -> String {
    format!(
        "{}: {}",
        error,
        match error {
            // ConfyError::BadTomlData(e)
            ConfyError::BadYamlData(e) => e.to_string(),
            // ConfyError::BadRonData(#[source] ron::error::SpannedError),
            ConfyError::DirectoryCreationFailed(e) => e.to_string(),
            ConfyError::GeneralLoadError(e) => e.to_string(),
            ConfyError::BadConfigDirectory(e) => e.to_string(),
            // ConfyError::SerializeTomlError(e) => e.to_string(),
            ConfyError::SerializeYamlError(e) => e.to_string(),
            // ConfyError::SerializeRonError() => e.to_string(),
            ConfyError::WriteConfigurationFileError(e) => e.to_string(),
            ConfyError::ReadConfigurationFileError(e) => e.to_string(),
            ConfyError::OpenConfigurationFileError(e) => e.to_string(),
            ConfyError::SetPermissionsFileError(e) => e.to_string(),
        }
    )
}
