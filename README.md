# Mixamo to GLB Converter

A tool for converting Mixamo FBX character models and animations into a single GLB file.

## Features

- Import FBX files from folders or zip archives
- Automatically detects models vs animations based on file size
- Combines multiple animations into a single GLB file
- Preserves skeleton hierarchy, skinning, and inverse bind matrices
- Optional root motion stripping
- GUI with real-time 3D preview
- CLI mode for batch processing
- Detailed conversion reports

## Usage

### GUI Mode

```bash
cargo run --release
```

Drop a folder or zip file containing Mixamo FBX files onto the window, or use the Import buttons.

### CLI Mode

```bash
cargo run --release -- "path/to/mixamo/files.zip"
```

The CLI automatically:
- Extracts zip files to a temp directory
- Loads all FBX files (largest as model, rest as animations)
- Exports to `<input_name>.glb` in the current directory
- Generates a detailed report in the `reports/` folder

## Input Format

The tool expects a folder or zip containing:
- One character model FBX (typically the largest file)
- Multiple animation FBX files from Mixamo

All animations should be rigged to the same skeleton as the model.

## Output

- Single `.glb` file containing:
  - Skinned mesh with all vertex attributes
  - Complete skeleton hierarchy
  - All selected animations
  - Inverse bind matrices for proper skinning

## Building

```bash
cargo build --release
```

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
