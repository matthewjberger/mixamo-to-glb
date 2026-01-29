use chrono::Local;
use nightshade::ecs::animation::components::{
    AnimationClip, AnimationProperty, AnimationSamplerOutput,
};
use nightshade::ecs::prefab::import_gltf_from_path;
use nightshade::ecs::prefab::resources::mesh_cache_insert;
use nightshade::ecs::prefab::{GltfSkin, Prefab, PrefabNode};
use nightshade::prelude::*;
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use walkdir::WalkDir;

const FBX_TO_GLTF_SCALE: f32 = 0.01;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();

    if args.len() > 1 {
        run_cli(&args[1..])?;
    } else {
        launch(MixamoConverter::default())?;
    }
    Ok(())
}

fn run_cli(args: &[String]) -> Result<(), Box<dyn std::error::Error>> {
    let input_path = PathBuf::from(&args[0]);
    let output_path = if args.len() > 1 {
        PathBuf::from(&args[1])
    } else {
        let stem = input_path.file_stem().unwrap_or_default().to_string_lossy();
        PathBuf::from(format!("{}.glb", stem))
    };

    println!("Mixamo to GLB Converter (CLI Mode)");
    println!("Input: {}", input_path.display());
    println!("Output: {}", output_path.display());

    let mut converter = CliConverter::new();
    converter.import_path(&input_path)?;
    converter.export_glb(&output_path)?;

    Ok(())
}

struct CliConverter {
    fbx_files: Vec<FbxFileInfo>,
    loaded_models: Vec<LoadedModel>,
    loaded_animations: Vec<LoadedAnimation>,
    temp_dir: Option<PathBuf>,
    strip_root_motion: bool,
    report_lines: Vec<String>,
}

impl CliConverter {
    fn new() -> Self {
        Self {
            fbx_files: Vec::new(),
            loaded_models: Vec::new(),
            loaded_animations: Vec::new(),
            temp_dir: None,
            strip_root_motion: true,
            report_lines: Vec::new(),
        }
    }

    fn log(&mut self, message: &str) {
        let timestamp = Local::now().format("%H:%M:%S").to_string();
        println!("[{}] {}", timestamp, message);
        self.report_lines
            .push(format!("[{}] {}", timestamp, message));
    }

    fn import_path(&mut self, path: &Path) -> Result<(), Box<dyn std::error::Error>> {
        self.log(&format!("Starting import from: {}", path.display()));

        let source_path = if path.extension().is_some_and(|e| e == "zip") {
            self.log("Extracting zip file...");
            self.extract_zip(path)?
        } else {
            path.to_path_buf()
        };

        self.log(&format!(
            "Scanning for FBX files in: {}",
            source_path.display()
        ));
        self.scan_for_fbx_files(&source_path);

        if self.fbx_files.is_empty() {
            return Err("No FBX files found".into());
        }

        self.log(&format!("Found {} FBX files", self.fbx_files.len()));
        self.load_fbx_files();

        self.log(&format!(
            "Loaded {} models and {} animations",
            self.loaded_models.len(),
            self.loaded_animations.len()
        ));

        Ok(())
    }

    fn extract_zip(&mut self, zip_path: &Path) -> Result<PathBuf, Box<dyn std::error::Error>> {
        let temp_dir = std::env::temp_dir().join(format!("mixamo_to_glb_{}", std::process::id()));
        std::fs::create_dir_all(&temp_dir)?;

        let file = std::fs::File::open(zip_path)?;
        let mut archive = zip::ZipArchive::new(file)?;

        for index in 0..archive.len() {
            let mut file = archive.by_index(index)?;
            let outpath = temp_dir.join(file.mangled_name());

            if file.is_dir() {
                std::fs::create_dir_all(&outpath)?;
            } else {
                if let Some(parent) = outpath.parent() {
                    std::fs::create_dir_all(parent)?;
                }
                let mut outfile = std::fs::File::create(&outpath)?;
                std::io::copy(&mut file, &mut outfile)?;
            }
        }

        self.temp_dir = Some(temp_dir.clone());
        self.log(&format!("Extracted to: {}", temp_dir.display()));
        Ok(temp_dir)
    }

    fn scan_for_fbx_files(&mut self, dir: &Path) {
        for entry in WalkDir::new(dir).into_iter().filter_map(|e| e.ok()) {
            let path = entry.path();
            if path
                .extension()
                .is_some_and(|e| e.eq_ignore_ascii_case("fbx"))
            {
                let metadata = std::fs::metadata(path).ok();
                let size_bytes = metadata.map_or(0, |m| m.len());
                let name = path
                    .file_stem()
                    .unwrap_or_default()
                    .to_string_lossy()
                    .to_string();

                let is_model = size_bytes > 1_000_000;

                self.fbx_files.push(FbxFileInfo {
                    path: path.to_path_buf(),
                    name: name.clone(),
                    size_bytes,
                    is_model,
                    selected: true,
                });

                self.log(&format!(
                    "  {} ({} KB) - {}",
                    name,
                    size_bytes / 1024,
                    if is_model { "MODEL" } else { "Animation" }
                ));
            }
        }

        self.fbx_files
            .sort_by(|a, b| b.size_bytes.cmp(&a.size_bytes));
    }

    fn load_fbx_files(&mut self) {
        let files: Vec<_> = self.fbx_files.clone();

        for file_info in &files {
            if file_info.is_model {
                self.log(&format!("Loading model: {}", file_info.name));
                match nightshade::ecs::prefab::import_fbx_from_path(&file_info.path) {
                    Ok(result) => {
                        self.log(&format!("  Meshes: {}", result.meshes.len()));
                        self.log(&format!("  Skins: {}", result.skins.len()));
                        self.log(&format!("  Textures: {}", result.textures.len()));
                        self.log(&format!("  Node count: {}", result.node_count));

                        if let Some(prefab) = result.prefabs.into_iter().next() {
                            self.loaded_models.push(LoadedModel {
                                name: file_info.name.clone(),
                                prefab,
                                skins: result.skins,
                                meshes: result.meshes,
                                textures: result.textures,
                                node_count: result.node_count,
                            });
                        }
                    }
                    Err(error) => {
                        self.log(&format!(
                            "Failed to load model {}: {}",
                            file_info.name, error
                        ));
                    }
                }
            } else {
                self.log(&format!("Loading animation: {}", file_info.name));
                match nightshade::ecs::prefab::import_fbx_animations_from_path(&file_info.path) {
                    Ok(clips) => {
                        if !clips.is_empty() {
                            self.log(&format!("  Clips: {}", clips.len()));
                            for clip in &clips {
                                self.log(&format!(
                                    "    {} - {} channels, {:.2}s duration",
                                    clip.name,
                                    clip.channels.len(),
                                    clip.duration
                                ));
                            }
                            self.loaded_animations.push(LoadedAnimation {
                                name: file_info.name.clone(),
                                clips,
                            });
                        }
                    }
                    Err(error) => {
                        self.log(&format!(
                            "Failed to load animation {}: {}",
                            file_info.name, error
                        ));
                    }
                }
            }
        }
    }

    fn export_glb(&mut self, output_path: &Path) -> Result<(), Box<dyn std::error::Error>> {
        if self.loaded_models.is_empty() {
            return Err("No models loaded".into());
        }

        let model_name = self.loaded_models[0].name.clone();
        self.log(&format!("Exporting model: {}", model_name));

        let mut all_animations: Vec<AnimationClip> = Vec::new();
        for file_info in &self.fbx_files {
            if !file_info.is_model && file_info.selected {
                if let Some(loaded_anim) = self
                    .loaded_animations
                    .iter()
                    .find(|a| a.name == file_info.name)
                {
                    for mut clip in loaded_anim.clips.clone() {
                        if self.strip_root_motion {
                            clip.channels.retain(|channel| {
                                channel.target_property != AnimationProperty::Translation
                            });
                        }
                        clip.name = file_info.name.clone();
                        all_animations.push(clip);
                    }
                }
            }
        }

        self.log(&format!("Including {} animations", all_animations.len()));

        let glb_data = self.build_glb_for_model(0, &all_animations)?;
        std::fs::write(output_path, &glb_data)?;
        self.log(&format!(
            "Wrote {} bytes to {}",
            glb_data.len(),
            output_path.display()
        ));

        self.write_report_for_model(output_path, 0, &all_animations)?;

        self.log("Verifying exported GLB...");
        self.verify_glb(output_path)?;

        Ok(())
    }

    fn verify_glb(&mut self, path: &Path) -> Result<(), Box<dyn std::error::Error>> {
        match import_gltf_from_path(path) {
            Ok(result) => {
                self.log("GLB Verification Results:");
                self.log(&format!("  Prefabs: {}", result.prefabs.len()));
                self.log(&format!("  Meshes: {}", result.meshes.len()));
                self.log(&format!("  Skins: {}", result.skins.len()));
                self.log(&format!("  Animations: {}", result.animations.len()));
                self.log(&format!("  Textures: {}", result.textures.len()));
                self.log(&format!("  Node count: {}", result.node_count));

                for (name, mesh) in &result.meshes {
                    let vertex_count = if let Some(ref skin_data) = mesh.skin_data {
                        skin_data.skinned_vertices.len()
                    } else {
                        mesh.vertices.len()
                    };
                    self.log(&format!(
                        "    Mesh '{}': {} vertices, {} indices",
                        name,
                        vertex_count,
                        mesh.indices.len()
                    ));
                }

                for (index, skin) in result.skins.iter().enumerate() {
                    self.log(&format!(
                        "    Skin {}: {} joints, {} IBMs",
                        index,
                        skin.joints.len(),
                        skin.inverse_bind_matrices.len()
                    ));
                }

                for anim in &result.animations {
                    self.log(&format!(
                        "    Animation '{}': {} channels, {:.2}s",
                        anim.name,
                        anim.channels.len(),
                        anim.duration
                    ));
                }

                if result.meshes.is_empty() {
                    self.log("WARNING: No meshes found in exported GLB!");
                    return Err("GLB verification failed: no meshes".into());
                }

                self.log("GLB verification passed!");
                Ok(())
            }
            Err(error) => {
                self.log(&format!("GLB verification FAILED: {}", error));
                Err(error)
            }
        }
    }

    fn build_glb_for_model(
        &mut self,
        model_index: usize,
        animations: &[AnimationClip],
    ) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
        let model = LoadedModel {
            name: self.loaded_models[model_index].name.clone(),
            prefab: self.loaded_models[model_index].prefab.clone(),
            skins: self.loaded_models[model_index].skins.clone(),
            meshes: self.loaded_models[model_index].meshes.clone(),
            textures: self.loaded_models[model_index].textures.clone(),
            node_count: self.loaded_models[model_index].node_count,
        };
        self.build_glb_internal(&model, animations)
    }

    fn write_report_for_model(
        &self,
        output_path: &Path,
        model_index: usize,
        animations: &[AnimationClip],
    ) -> Result<(), Box<dyn std::error::Error>> {
        let model = &self.loaded_models[model_index];
        self.write_report(output_path, model, animations)
    }

    fn build_glb_internal(
        &mut self,
        model: &LoadedModel,
        animations: &[AnimationClip],
    ) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
        use gltf_json as json;
        use json::validation::USize64;

        let mut buffer_data: Vec<u8> = Vec::new();

        let mut accessors: Vec<json::Accessor> = Vec::new();
        let mut buffer_views: Vec<json::buffer::View> = Vec::new();
        let mut meshes: Vec<json::Mesh> = Vec::new();
        let mut gltf_nodes: Vec<json::Node> = Vec::new();
        let mut skins: Vec<json::Skin> = Vec::new();
        let mut images: Vec<json::Image> = Vec::new();
        let mut gltf_textures: Vec<json::Texture> = Vec::new();
        let mut materials: Vec<json::Material> = Vec::new();
        let mut samplers: Vec<json::texture::Sampler> = Vec::new();
        let mut texture_name_to_index: HashMap<String, u32> = HashMap::new();

        let mut node_index_map: HashMap<usize, usize> = HashMap::new();

        struct PrefabNodeInfo {
            gltf_index: usize,
            child_gltf_indices: Vec<usize>,
            name: Option<String>,
            translation: [f32; 3],
            rotation: [f32; 4],
            scale: [f32; 3],
        }

        fn assign_indices(
            prefab_node: &PrefabNode,
            node_index_map: &mut HashMap<usize, usize>,
            next_index: &mut usize,
            node_infos: &mut Vec<PrefabNodeInfo>,
        ) -> usize {
            let current_index = *next_index;
            *next_index += 1;

            if let Some(prefab_idx) = prefab_node.node_index {
                node_index_map.insert(prefab_idx, current_index);
            }

            let mut child_gltf_indices = Vec::new();
            for child in &prefab_node.children {
                let child_index = assign_indices(child, node_index_map, next_index, node_infos);
                child_gltf_indices.push(child_index);
            }

            node_infos.push(PrefabNodeInfo {
                gltf_index: current_index,
                child_gltf_indices,
                name: prefab_node.components.name.as_ref().map(|n| n.0.clone()),
                translation: [
                    prefab_node.local_transform.translation.x * FBX_TO_GLTF_SCALE,
                    prefab_node.local_transform.translation.y * FBX_TO_GLTF_SCALE,
                    prefab_node.local_transform.translation.z * FBX_TO_GLTF_SCALE,
                ],
                rotation: [
                    prefab_node.local_transform.rotation.i,
                    prefab_node.local_transform.rotation.j,
                    prefab_node.local_transform.rotation.k,
                    prefab_node.local_transform.rotation.w,
                ],
                scale: [
                    prefab_node.local_transform.scale.x,
                    prefab_node.local_transform.scale.y,
                    prefab_node.local_transform.scale.z,
                ],
            });

            current_index
        }

        let mut node_infos: Vec<PrefabNodeInfo> = Vec::new();
        let mut next_index = 0usize;

        for root_node in &model.prefab.root_nodes {
            assign_indices(
                root_node,
                &mut node_index_map,
                &mut next_index,
                &mut node_infos,
            );
        }

        node_infos.sort_by_key(|info| info.gltf_index);

        for info in &node_infos {
            gltf_nodes.push(json::Node {
                name: info.name.clone(),
                translation: Some(info.translation),
                rotation: Some(json::scene::UnitQuaternion(info.rotation)),
                scale: Some(info.scale),
                mesh: None,
                skin: None,
                children: if info.child_gltf_indices.is_empty() {
                    None
                } else {
                    Some(
                        info.child_gltf_indices
                            .iter()
                            .map(|i| json::Index::new(*i as u32))
                            .collect(),
                    )
                },
                ..Default::default()
            });
        }

        self.log(&format!(
            "Built {} nodes from prefab hierarchy",
            gltf_nodes.len()
        ));
        self.log(&format!("Node index mapping: {:?}", node_index_map));

        if !model.textures.is_empty() {
            samplers.push(json::texture::Sampler {
                mag_filter: Some(json::validation::Checked::Valid(
                    json::texture::MagFilter::Linear,
                )),
                min_filter: Some(json::validation::Checked::Valid(
                    json::texture::MinFilter::LinearMipmapLinear,
                )),
                wrap_s: json::validation::Checked::Valid(json::texture::WrappingMode::Repeat),
                wrap_t: json::validation::Checked::Valid(json::texture::WrappingMode::Repeat),
                name: None,
                extensions: None,
                extras: Default::default(),
            });
        }

        for (texture_name, (rgba_data, width, height)) in &model.textures {
            self.log(&format!(
                "Exporting texture: {} ({}x{})",
                texture_name, width, height
            ));

            let png_data = {
                let mut png_buffer = Vec::new();
                let mut cursor = std::io::Cursor::new(&mut png_buffer);
                let encoder = image::codecs::png::PngEncoder::new(&mut cursor);
                image::ImageEncoder::write_image(
                    encoder,
                    rgba_data,
                    *width,
                    *height,
                    image::ExtendedColorType::Rgba8,
                )?;
                png_buffer
            };

            while buffer_data.len() % 4 != 0 {
                buffer_data.push(0);
            }

            let image_start = buffer_data.len();
            buffer_data.extend_from_slice(&png_data);
            let image_length = buffer_data.len() - image_start;

            buffer_views.push(json::buffer::View {
                buffer: json::Index::new(0),
                byte_offset: Some(USize64::from(image_start)),
                byte_length: USize64::from(image_length),
                byte_stride: None,
                target: None,
                name: None,
                extensions: None,
                extras: Default::default(),
            });

            let image_index = images.len() as u32;
            images.push(json::Image {
                buffer_view: Some(json::Index::new(buffer_views.len() as u32 - 1)),
                mime_type: Some(json::image::MimeType("image/png".to_string())),
                uri: None,
                name: Some(texture_name.clone()),
                extensions: None,
                extras: Default::default(),
            });

            let texture_index = gltf_textures.len() as u32;
            gltf_textures.push(json::Texture {
                sampler: Some(json::Index::new(0)),
                source: json::Index::new(image_index),
                name: Some(texture_name.clone()),
                extensions: None,
                extras: Default::default(),
            });

            texture_name_to_index.insert(texture_name.clone(), texture_index);
        }

        let default_material_index = if !model.textures.is_empty() {
            let base_color_texture = model
                .textures
                .keys()
                .next()
                .map(|name| texture_name_to_index.get(name).copied())
                .flatten();

            materials.push(json::Material {
                name: Some("DefaultMaterial".to_string()),
                pbr_metallic_roughness: json::material::PbrMetallicRoughness {
                    base_color_factor: json::material::PbrBaseColorFactor([1.0, 1.0, 1.0, 1.0]),
                    base_color_texture: base_color_texture.map(|idx| json::texture::Info {
                        index: json::Index::new(idx),
                        tex_coord: 0,
                        extensions: None,
                        extras: Default::default(),
                    }),
                    metallic_factor: json::material::StrengthFactor(0.0),
                    roughness_factor: json::material::StrengthFactor(0.5),
                    metallic_roughness_texture: None,
                    extensions: None,
                    extras: Default::default(),
                },
                alpha_mode: json::validation::Checked::Valid(json::material::AlphaMode::Opaque),
                alpha_cutoff: None,
                double_sided: false,
                normal_texture: None,
                occlusion_texture: None,
                emissive_texture: None,
                emissive_factor: json::material::EmissiveFactor([0.0, 0.0, 0.0]),
                extensions: None,
                extras: Default::default(),
            });
            Some(0u32)
        } else {
            materials.push(json::Material {
                name: Some("DefaultMaterial".to_string()),
                pbr_metallic_roughness: json::material::PbrMetallicRoughness {
                    base_color_factor: json::material::PbrBaseColorFactor([0.8, 0.8, 0.8, 1.0]),
                    base_color_texture: None,
                    metallic_factor: json::material::StrengthFactor(0.0),
                    roughness_factor: json::material::StrengthFactor(0.5),
                    metallic_roughness_texture: None,
                    extensions: None,
                    extras: Default::default(),
                },
                alpha_mode: json::validation::Checked::Valid(json::material::AlphaMode::Opaque),
                alpha_cutoff: None,
                double_sided: false,
                normal_texture: None,
                occlusion_texture: None,
                emissive_texture: None,
                emissive_factor: json::material::EmissiveFactor([0.0, 0.0, 0.0]),
                extensions: None,
                extras: Default::default(),
            });
            Some(0u32)
        };

        self.log(&format!(
            "Created {} textures and {} materials",
            gltf_textures.len(),
            materials.len()
        ));

        for (skin_idx, skin) in model.skins.iter().enumerate() {
            self.log(&format!(
                "Processing skin {} with {} joints",
                skin_idx,
                skin.joints.len()
            ));

            let ibm_start = buffer_data.len();
            for ibm in &skin.inverse_bind_matrices {
                for col in 0..4 {
                    for row in 0..4 {
                        let value = ibm[(row, col)];
                        let scaled_value = if col == 3 && row < 3 {
                            value * FBX_TO_GLTF_SCALE
                        } else {
                            value
                        };
                        buffer_data.extend_from_slice(&scaled_value.to_le_bytes());
                    }
                }
            }
            let ibm_length = buffer_data.len() - ibm_start;

            buffer_views.push(json::buffer::View {
                buffer: json::Index::new(0),
                byte_offset: Some(USize64::from(ibm_start)),
                byte_length: USize64::from(ibm_length),
                byte_stride: None,
                target: None,
                name: None,
                extensions: None,
                extras: Default::default(),
            });

            let ibm_accessor_index = accessors.len() as u32;
            accessors.push(json::Accessor {
                buffer_view: Some(json::Index::new(buffer_views.len() as u32 - 1)),
                byte_offset: Some(USize64::from(0usize)),
                count: USize64::from(skin.inverse_bind_matrices.len()),
                component_type: json::validation::Checked::Valid(
                    json::accessor::GenericComponentType(json::accessor::ComponentType::F32),
                ),
                type_: json::validation::Checked::Valid(json::accessor::Type::Mat4),
                min: None,
                max: None,
                name: None,
                normalized: false,
                sparse: None,
                extensions: None,
                extras: Default::default(),
            });

            let mut unmapped_joints = 0;
            let first_valid_node = node_index_map.values().next().copied().unwrap_or(0);

            let joint_indices: Vec<json::Index<json::Node>> = skin
                .joints
                .iter()
                .map(|&joint_idx| {
                    if let Some(&gltf_idx) = node_index_map.get(&joint_idx) {
                        json::Index::new(gltf_idx as u32)
                    } else {
                        unmapped_joints += 1;
                        json::Index::new(first_valid_node as u32)
                    }
                })
                .collect();

            if unmapped_joints > 0 {
                self.log(&format!(
                    "  WARNING: {} of {} joints could not be mapped to glTF nodes",
                    unmapped_joints,
                    skin.joints.len()
                ));
            }
            self.log(&format!(
                "  Mapped {} joints to glTF nodes",
                joint_indices.len()
            ));

            skins.push(json::Skin {
                inverse_bind_matrices: Some(json::Index::new(ibm_accessor_index)),
                joints: joint_indices,
                skeleton: None,
                name: skin.name.clone(),
                extensions: None,
                extras: Default::default(),
            });
        }

        for (mesh_name, mesh) in &model.meshes {
            self.log(&format!(
                "Processing mesh: {} ({} vertices, {} indices)",
                mesh_name,
                mesh.vertices.len(),
                mesh.indices.len()
            ));

            let has_skin = mesh.skin_data.is_some();
            self.log(&format!("  Has skin data: {}", has_skin));

            let (vertices_to_use, skinned_data) = if let Some(ref skin_data) = mesh.skin_data {
                self.log(&format!(
                    "  Skinned vertices: {}",
                    skin_data.skinned_vertices.len()
                ));
                (None, Some(skin_data))
            } else {
                (Some(&mesh.vertices), None)
            };

            let vertex_count =
                skinned_data.map_or_else(|| mesh.vertices.len(), |sd| sd.skinned_vertices.len());

            let positions_start = buffer_data.len();
            let mut min_pos = [f32::MAX; 3];
            let mut max_pos = [f32::MIN; 3];

            if let Some(sd) = skinned_data {
                for vertex in &sd.skinned_vertices {
                    let scaled_pos = [
                        vertex.position[0] * FBX_TO_GLTF_SCALE,
                        vertex.position[1] * FBX_TO_GLTF_SCALE,
                        vertex.position[2] * FBX_TO_GLTF_SCALE,
                    ];
                    buffer_data.extend_from_slice(&scaled_pos[0].to_le_bytes());
                    buffer_data.extend_from_slice(&scaled_pos[1].to_le_bytes());
                    buffer_data.extend_from_slice(&scaled_pos[2].to_le_bytes());
                    for idx in 0..3 {
                        min_pos[idx] = min_pos[idx].min(scaled_pos[idx]);
                        max_pos[idx] = max_pos[idx].max(scaled_pos[idx]);
                    }
                }
            } else if let Some(verts) = vertices_to_use {
                for vertex in verts {
                    let scaled_pos = [
                        vertex.position[0] * FBX_TO_GLTF_SCALE,
                        vertex.position[1] * FBX_TO_GLTF_SCALE,
                        vertex.position[2] * FBX_TO_GLTF_SCALE,
                    ];
                    buffer_data.extend_from_slice(&scaled_pos[0].to_le_bytes());
                    buffer_data.extend_from_slice(&scaled_pos[1].to_le_bytes());
                    buffer_data.extend_from_slice(&scaled_pos[2].to_le_bytes());
                    for idx in 0..3 {
                        min_pos[idx] = min_pos[idx].min(scaled_pos[idx]);
                        max_pos[idx] = max_pos[idx].max(scaled_pos[idx]);
                    }
                }
            }
            let positions_length = buffer_data.len() - positions_start;

            buffer_views.push(json::buffer::View {
                buffer: json::Index::new(0),
                byte_offset: Some(USize64::from(positions_start)),
                byte_length: USize64::from(positions_length),
                byte_stride: None,
                target: Some(json::validation::Checked::Valid(
                    json::buffer::Target::ArrayBuffer,
                )),
                name: None,
                extensions: None,
                extras: Default::default(),
            });

            let position_accessor_index = accessors.len() as u32;
            accessors.push(json::Accessor {
                buffer_view: Some(json::Index::new(buffer_views.len() as u32 - 1)),
                byte_offset: Some(USize64::from(0usize)),
                count: USize64::from(vertex_count),
                component_type: json::validation::Checked::Valid(
                    json::accessor::GenericComponentType(json::accessor::ComponentType::F32),
                ),
                type_: json::validation::Checked::Valid(json::accessor::Type::Vec3),
                min: Some(json::Value::Array(
                    min_pos.iter().map(|v| json::Value::from(*v)).collect(),
                )),
                max: Some(json::Value::Array(
                    max_pos.iter().map(|v| json::Value::from(*v)).collect(),
                )),
                name: None,
                normalized: false,
                sparse: None,
                extensions: None,
                extras: Default::default(),
            });

            let normals_start = buffer_data.len();
            if let Some(sd) = skinned_data {
                for vertex in &sd.skinned_vertices {
                    buffer_data.extend_from_slice(&vertex.normal[0].to_le_bytes());
                    buffer_data.extend_from_slice(&vertex.normal[1].to_le_bytes());
                    buffer_data.extend_from_slice(&vertex.normal[2].to_le_bytes());
                }
            } else if let Some(verts) = vertices_to_use {
                for vertex in verts {
                    buffer_data.extend_from_slice(&vertex.normal[0].to_le_bytes());
                    buffer_data.extend_from_slice(&vertex.normal[1].to_le_bytes());
                    buffer_data.extend_from_slice(&vertex.normal[2].to_le_bytes());
                }
            }
            let normals_length = buffer_data.len() - normals_start;

            buffer_views.push(json::buffer::View {
                buffer: json::Index::new(0),
                byte_offset: Some(USize64::from(normals_start)),
                byte_length: USize64::from(normals_length),
                byte_stride: None,
                target: Some(json::validation::Checked::Valid(
                    json::buffer::Target::ArrayBuffer,
                )),
                name: None,
                extensions: None,
                extras: Default::default(),
            });

            let normal_accessor_index = accessors.len() as u32;
            accessors.push(json::Accessor {
                buffer_view: Some(json::Index::new(buffer_views.len() as u32 - 1)),
                byte_offset: Some(USize64::from(0usize)),
                count: USize64::from(vertex_count),
                component_type: json::validation::Checked::Valid(
                    json::accessor::GenericComponentType(json::accessor::ComponentType::F32),
                ),
                type_: json::validation::Checked::Valid(json::accessor::Type::Vec3),
                min: None,
                max: None,
                name: None,
                normalized: false,
                sparse: None,
                extensions: None,
                extras: Default::default(),
            });

            let texcoords_start = buffer_data.len();
            if let Some(sd) = skinned_data {
                for vertex in &sd.skinned_vertices {
                    buffer_data.extend_from_slice(&vertex.tex_coords[0].to_le_bytes());
                    buffer_data.extend_from_slice(&vertex.tex_coords[1].to_le_bytes());
                }
            } else if let Some(verts) = vertices_to_use {
                for vertex in verts {
                    buffer_data.extend_from_slice(&vertex.tex_coords[0].to_le_bytes());
                    buffer_data.extend_from_slice(&vertex.tex_coords[1].to_le_bytes());
                }
            }
            let texcoords_length = buffer_data.len() - texcoords_start;

            buffer_views.push(json::buffer::View {
                buffer: json::Index::new(0),
                byte_offset: Some(USize64::from(texcoords_start)),
                byte_length: USize64::from(texcoords_length),
                byte_stride: None,
                target: Some(json::validation::Checked::Valid(
                    json::buffer::Target::ArrayBuffer,
                )),
                name: None,
                extensions: None,
                extras: Default::default(),
            });

            let texcoord_accessor_index = accessors.len() as u32;
            accessors.push(json::Accessor {
                buffer_view: Some(json::Index::new(buffer_views.len() as u32 - 1)),
                byte_offset: Some(USize64::from(0usize)),
                count: USize64::from(vertex_count),
                component_type: json::validation::Checked::Valid(
                    json::accessor::GenericComponentType(json::accessor::ComponentType::F32),
                ),
                type_: json::validation::Checked::Valid(json::accessor::Type::Vec2),
                min: None,
                max: None,
                name: None,
                normalized: false,
                sparse: None,
                extensions: None,
                extras: Default::default(),
            });

            let mut joints_accessor_index = None;
            let mut weights_accessor_index = None;

            if let Some(sd) = skinned_data {
                let joints_start = buffer_data.len();
                for vertex in &sd.skinned_vertices {
                    buffer_data.extend_from_slice(&(vertex.joint_indices[0] as u16).to_le_bytes());
                    buffer_data.extend_from_slice(&(vertex.joint_indices[1] as u16).to_le_bytes());
                    buffer_data.extend_from_slice(&(vertex.joint_indices[2] as u16).to_le_bytes());
                    buffer_data.extend_from_slice(&(vertex.joint_indices[3] as u16).to_le_bytes());
                }
                let joints_length = buffer_data.len() - joints_start;

                buffer_views.push(json::buffer::View {
                    buffer: json::Index::new(0),
                    byte_offset: Some(USize64::from(joints_start)),
                    byte_length: USize64::from(joints_length),
                    byte_stride: None,
                    target: Some(json::validation::Checked::Valid(
                        json::buffer::Target::ArrayBuffer,
                    )),
                    name: None,
                    extensions: None,
                    extras: Default::default(),
                });

                joints_accessor_index = Some(accessors.len() as u32);
                accessors.push(json::Accessor {
                    buffer_view: Some(json::Index::new(buffer_views.len() as u32 - 1)),
                    byte_offset: Some(USize64::from(0usize)),
                    count: USize64::from(sd.skinned_vertices.len()),
                    component_type: json::validation::Checked::Valid(
                        json::accessor::GenericComponentType(json::accessor::ComponentType::U16),
                    ),
                    type_: json::validation::Checked::Valid(json::accessor::Type::Vec4),
                    min: None,
                    max: None,
                    name: None,
                    normalized: false,
                    sparse: None,
                    extensions: None,
                    extras: Default::default(),
                });

                let weights_start = buffer_data.len();
                for vertex in &sd.skinned_vertices {
                    buffer_data.extend_from_slice(&vertex.joint_weights[0].to_le_bytes());
                    buffer_data.extend_from_slice(&vertex.joint_weights[1].to_le_bytes());
                    buffer_data.extend_from_slice(&vertex.joint_weights[2].to_le_bytes());
                    buffer_data.extend_from_slice(&vertex.joint_weights[3].to_le_bytes());
                }
                let weights_length = buffer_data.len() - weights_start;

                buffer_views.push(json::buffer::View {
                    buffer: json::Index::new(0),
                    byte_offset: Some(USize64::from(weights_start)),
                    byte_length: USize64::from(weights_length),
                    byte_stride: None,
                    target: Some(json::validation::Checked::Valid(
                        json::buffer::Target::ArrayBuffer,
                    )),
                    name: None,
                    extensions: None,
                    extras: Default::default(),
                });

                weights_accessor_index = Some(accessors.len() as u32);
                accessors.push(json::Accessor {
                    buffer_view: Some(json::Index::new(buffer_views.len() as u32 - 1)),
                    byte_offset: Some(USize64::from(0usize)),
                    count: USize64::from(sd.skinned_vertices.len()),
                    component_type: json::validation::Checked::Valid(
                        json::accessor::GenericComponentType(json::accessor::ComponentType::F32),
                    ),
                    type_: json::validation::Checked::Valid(json::accessor::Type::Vec4),
                    min: None,
                    max: None,
                    name: None,
                    normalized: false,
                    sparse: None,
                    extensions: None,
                    extras: Default::default(),
                });
            }

            let indices_start = buffer_data.len();
            for index in &mesh.indices {
                buffer_data.extend_from_slice(&(*index).to_le_bytes());
            }
            let indices_length = buffer_data.len() - indices_start;

            buffer_views.push(json::buffer::View {
                buffer: json::Index::new(0),
                byte_offset: Some(USize64::from(indices_start)),
                byte_length: USize64::from(indices_length),
                byte_stride: None,
                target: Some(json::validation::Checked::Valid(
                    json::buffer::Target::ElementArrayBuffer,
                )),
                name: None,
                extensions: None,
                extras: Default::default(),
            });

            let indices_accessor_index = accessors.len() as u32;
            accessors.push(json::Accessor {
                buffer_view: Some(json::Index::new(buffer_views.len() as u32 - 1)),
                byte_offset: Some(USize64::from(0usize)),
                count: USize64::from(mesh.indices.len()),
                component_type: json::validation::Checked::Valid(
                    json::accessor::GenericComponentType(json::accessor::ComponentType::U32),
                ),
                type_: json::validation::Checked::Valid(json::accessor::Type::Scalar),
                min: None,
                max: None,
                name: None,
                normalized: false,
                sparse: None,
                extensions: None,
                extras: Default::default(),
            });

            let mut attributes = std::collections::BTreeMap::new();
            attributes.insert(
                json::validation::Checked::Valid(json::mesh::Semantic::Positions),
                json::Index::new(position_accessor_index),
            );
            attributes.insert(
                json::validation::Checked::Valid(json::mesh::Semantic::Normals),
                json::Index::new(normal_accessor_index),
            );
            attributes.insert(
                json::validation::Checked::Valid(json::mesh::Semantic::TexCoords(0)),
                json::Index::new(texcoord_accessor_index),
            );

            if let Some(joints_idx) = joints_accessor_index {
                attributes.insert(
                    json::validation::Checked::Valid(json::mesh::Semantic::Joints(0)),
                    json::Index::new(joints_idx),
                );
            }
            if let Some(weights_idx) = weights_accessor_index {
                attributes.insert(
                    json::validation::Checked::Valid(json::mesh::Semantic::Weights(0)),
                    json::Index::new(weights_idx),
                );
            }

            meshes.push(json::Mesh {
                primitives: vec![json::mesh::Primitive {
                    attributes,
                    indices: Some(json::Index::new(indices_accessor_index)),
                    material: default_material_index.map(|idx| json::Index::new(idx)),
                    mode: json::validation::Checked::Valid(json::mesh::Mode::Triangles),
                    targets: None,
                    extensions: None,
                    extras: Default::default(),
                }],
                name: Some(mesh_name.clone()),
                weights: None,
                extensions: None,
                extras: Default::default(),
            });

            let mesh_node_index = gltf_nodes.len();
            let skin_index = if skinned_data.is_some() {
                skinned_data.and_then(|sd| sd.skin_index).or(Some(0))
            } else {
                None
            };

            gltf_nodes.push(json::Node {
                mesh: Some(json::Index::new(meshes.len() as u32 - 1)),
                skin: skin_index.map(|idx| json::Index::new(idx as u32)),
                name: Some(mesh_name.clone()),
                ..Default::default()
            });

            self.log(&format!(
                "  Created mesh node at index {} with skin {:?}",
                mesh_node_index, skin_index
            ));
        }

        let mut gltf_animations: Vec<json::Animation> = Vec::new();

        let mut node_name_to_gltf_index: HashMap<String, usize> = HashMap::new();
        for (gltf_idx, node) in gltf_nodes.iter().enumerate() {
            if let Some(ref name) = node.name {
                node_name_to_gltf_index.insert(name.clone(), gltf_idx);
            }
        }
        self.log(&format!(
            "Built node name map with {} entries",
            node_name_to_gltf_index.len()
        ));

        for anim in animations {
            self.log(&format!(
                "Processing animation: {} ({} channels)",
                anim.name,
                anim.channels.len()
            ));

            let mut samplers: Vec<json::animation::Sampler> = Vec::new();
            let mut channels: Vec<json::animation::Channel> = Vec::new();
            let mut unmapped_channels = 0;

            for channel in &anim.channels {
                let target_node_idx = if let Some(ref target_name) = channel.target_bone_name {
                    node_name_to_gltf_index.get(target_name).copied()
                } else {
                    node_index_map.get(&channel.target_node).copied()
                };

                let Some(target_node_idx) = target_node_idx else {
                    unmapped_channels += 1;
                    continue;
                };

                let times_start = buffer_data.len();
                for time in &channel.sampler.input {
                    buffer_data.extend_from_slice(&time.to_le_bytes());
                }
                let times_length = buffer_data.len() - times_start;

                buffer_views.push(json::buffer::View {
                    buffer: json::Index::new(0),
                    byte_offset: Some(USize64::from(times_start)),
                    byte_length: USize64::from(times_length),
                    byte_stride: None,
                    target: None,
                    name: None,
                    extensions: None,
                    extras: Default::default(),
                });

                let time_accessor_index = accessors.len() as u32;
                let min_time = channel.sampler.input.first().copied().unwrap_or(0.0);
                let max_time = channel.sampler.input.last().copied().unwrap_or(0.0);

                accessors.push(json::Accessor {
                    buffer_view: Some(json::Index::new(buffer_views.len() as u32 - 1)),
                    byte_offset: Some(USize64::from(0usize)),
                    count: USize64::from(channel.sampler.input.len()),
                    component_type: json::validation::Checked::Valid(
                        json::accessor::GenericComponentType(json::accessor::ComponentType::F32),
                    ),
                    type_: json::validation::Checked::Valid(json::accessor::Type::Scalar),
                    min: Some(json::Value::Array(vec![json::Value::from(min_time)])),
                    max: Some(json::Value::Array(vec![json::Value::from(max_time)])),
                    name: None,
                    normalized: false,
                    sparse: None,
                    extensions: None,
                    extras: Default::default(),
                });

                let values_start = buffer_data.len();
                let (accessor_type, path, value_count) =
                    match (&channel.target_property, &channel.sampler.output) {
                        (AnimationProperty::Translation, AnimationSamplerOutput::Vec3(values)) => {
                            for v in values {
                                buffer_data.extend_from_slice(&(v.x * FBX_TO_GLTF_SCALE).to_le_bytes());
                                buffer_data.extend_from_slice(&(v.y * FBX_TO_GLTF_SCALE).to_le_bytes());
                                buffer_data.extend_from_slice(&(v.z * FBX_TO_GLTF_SCALE).to_le_bytes());
                            }
                            (
                                json::accessor::Type::Vec3,
                                json::animation::Property::Translation,
                                values.len(),
                            )
                        }
                        (AnimationProperty::Rotation, AnimationSamplerOutput::Quat(values)) => {
                            for q in values {
                                buffer_data.extend_from_slice(&q.i.to_le_bytes());
                                buffer_data.extend_from_slice(&q.j.to_le_bytes());
                                buffer_data.extend_from_slice(&q.k.to_le_bytes());
                                buffer_data.extend_from_slice(&q.w.to_le_bytes());
                            }
                            (
                                json::accessor::Type::Vec4,
                                json::animation::Property::Rotation,
                                values.len(),
                            )
                        }
                        (AnimationProperty::Scale, AnimationSamplerOutput::Vec3(values)) => {
                            for v in values {
                                buffer_data.extend_from_slice(&v.x.to_le_bytes());
                                buffer_data.extend_from_slice(&v.y.to_le_bytes());
                                buffer_data.extend_from_slice(&v.z.to_le_bytes());
                            }
                            (
                                json::accessor::Type::Vec3,
                                json::animation::Property::Scale,
                                values.len(),
                            )
                        }
                        _ => continue,
                    };
                let values_length = buffer_data.len() - values_start;

                buffer_views.push(json::buffer::View {
                    buffer: json::Index::new(0),
                    byte_offset: Some(USize64::from(values_start)),
                    byte_length: USize64::from(values_length),
                    byte_stride: None,
                    target: None,
                    name: None,
                    extensions: None,
                    extras: Default::default(),
                });

                let value_accessor_index = accessors.len() as u32;
                accessors.push(json::Accessor {
                    buffer_view: Some(json::Index::new(buffer_views.len() as u32 - 1)),
                    byte_offset: Some(USize64::from(0usize)),
                    count: USize64::from(value_count),
                    component_type: json::validation::Checked::Valid(
                        json::accessor::GenericComponentType(json::accessor::ComponentType::F32),
                    ),
                    type_: json::validation::Checked::Valid(accessor_type),
                    min: None,
                    max: None,
                    name: None,
                    normalized: false,
                    sparse: None,
                    extensions: None,
                    extras: Default::default(),
                });

                let sampler_index = samplers.len();
                samplers.push(json::animation::Sampler {
                    input: json::Index::new(time_accessor_index),
                    output: json::Index::new(value_accessor_index),
                    interpolation: json::validation::Checked::Valid(
                        json::animation::Interpolation::Linear,
                    ),
                    extensions: None,
                    extras: Default::default(),
                });

                channels.push(json::animation::Channel {
                    sampler: json::Index::new(sampler_index as u32),
                    target: json::animation::Target {
                        node: json::Index::new(target_node_idx as u32),
                        path: json::validation::Checked::Valid(path),
                        extensions: None,
                        extras: Default::default(),
                    },
                    extensions: None,
                    extras: Default::default(),
                });
            }

            if unmapped_channels > 0 {
                self.log(&format!(
                    "  WARNING: {} channels could not be mapped to nodes",
                    unmapped_channels
                ));
            }

            if !channels.is_empty() {
                gltf_animations.push(json::Animation {
                    name: Some(anim.name.clone()),
                    channels,
                    samplers,
                    extensions: None,
                    extras: Default::default(),
                });
                self.log(&format!(
                    "  Added animation with {} channels",
                    gltf_animations.last().unwrap().channels.len()
                ));
            }
        }

        while !buffer_data.len().is_multiple_of(4) {
            buffer_data.push(0);
        }

        let root_nodes: Vec<json::Index<json::Node>> = (0..model.prefab.root_nodes.len())
            .map(|i| json::Index::new(i as u32))
            .collect();

        let mesh_root_start = gltf_nodes.len() - model.meshes.len();
        let mesh_node_indices: Vec<json::Index<json::Node>> = (mesh_root_start..gltf_nodes.len())
            .map(|i| json::Index::new(i as u32))
            .collect();

        let all_root_nodes: Vec<json::Index<json::Node>> = root_nodes
            .into_iter()
            .chain(mesh_node_indices.into_iter())
            .collect();

        let root = json::Root {
            asset: json::Asset {
                generator: Some("mixamo-to-glb".to_string()),
                version: "2.0".to_string(),
                ..Default::default()
            },
            accessors,
            buffer_views,
            buffers: vec![json::Buffer {
                byte_length: USize64::from(buffer_data.len()),
                uri: None,
                name: None,
                extensions: None,
                extras: Default::default(),
            }],
            images,
            samplers,
            textures: gltf_textures,
            materials,
            meshes,
            nodes: gltf_nodes,
            skins,
            animations: gltf_animations,
            scenes: vec![json::Scene {
                nodes: all_root_nodes,
                name: Some("Scene".to_string()),
                extensions: None,
                extras: Default::default(),
            }],
            scene: Some(json::Index::new(0)),
            ..Default::default()
        };

        let json_string = json::serialize::to_string_pretty(&root)?;
        let json_bytes = json_string.as_bytes();

        let mut json_chunk = json_bytes.to_vec();
        while !json_chunk.len().is_multiple_of(4) {
            json_chunk.push(0x20);
        }

        let mut glb: Vec<u8> = Vec::new();

        glb.extend_from_slice(b"glTF");
        glb.extend_from_slice(&2u32.to_le_bytes());
        let total_length = 12 + 8 + json_chunk.len() + 8 + buffer_data.len();
        glb.extend_from_slice(&(total_length as u32).to_le_bytes());

        glb.extend_from_slice(&(json_chunk.len() as u32).to_le_bytes());
        glb.extend_from_slice(&0x4E4F534Au32.to_le_bytes());
        glb.extend_from_slice(&json_chunk);

        glb.extend_from_slice(&(buffer_data.len() as u32).to_le_bytes());
        glb.extend_from_slice(&0x004E4942u32.to_le_bytes());
        glb.extend_from_slice(&buffer_data);

        self.log(&format!("GLB Summary:"));
        self.log(&format!("  Total size: {} bytes", glb.len()));
        self.log(&format!("  JSON chunk: {} bytes", json_chunk.len()));
        self.log(&format!("  Binary chunk: {} bytes", buffer_data.len()));
        self.log(&format!("  Nodes: {}", root.nodes.len()));
        self.log(&format!("  Meshes: {}", root.meshes.len()));
        self.log(&format!("  Skins: {}", root.skins.len()));
        self.log(&format!("  Animations: {}", root.animations.len()));
        self.log(&format!("  Images: {}", root.images.len()));
        self.log(&format!("  Textures: {}", root.textures.len()));
        self.log(&format!("  Materials: {}", root.materials.len()));
        self.log(&format!("  Accessors: {}", root.accessors.len()));
        self.log(&format!("  Buffer views: {}", root.buffer_views.len()));

        Ok(glb)
    }

    fn write_report(
        &self,
        output_path: &Path,
        model: &LoadedModel,
        animations: &[AnimationClip],
    ) -> Result<(), Box<dyn std::error::Error>> {
        let report_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("reports");
        std::fs::create_dir_all(&report_dir)?;

        let timestamp = Local::now().format("%Y%m%d_%H%M%S").to_string();
        let report_name = format!("{}_{}.txt", model.name, timestamp);
        let report_path = report_dir.join(&report_name);

        let mut report = String::new();
        report.push_str(
            "================================================================================\n",
        );
        report.push_str("                     MIXAMO TO GLB CONVERSION REPORT\n");
        report.push_str(
            "================================================================================\n\n",
        );

        report.push_str(&format!(
            "Generated: {}\n",
            Local::now().format("%Y-%m-%d %H:%M:%S")
        ));
        report.push_str(&format!("Output: {}\n\n", output_path.display()));

        report.push_str(
            "--------------------------------------------------------------------------------\n",
        );
        report.push_str("                              SOURCE FILES\n");
        report.push_str(
            "--------------------------------------------------------------------------------\n\n",
        );

        report.push_str("Models:\n");
        for file in &self.fbx_files {
            if file.is_model {
                report.push_str(&format!(
                    "  - {} ({} KB)\n",
                    file.name,
                    file.size_bytes / 1024
                ));
            }
        }

        report.push_str("\nAnimations:\n");
        for file in &self.fbx_files {
            if !file.is_model {
                let status = if file.selected {
                    "[INCLUDED]"
                } else {
                    "[EXCLUDED]"
                };
                report.push_str(&format!(
                    "  {} {} ({} KB)\n",
                    status,
                    file.name,
                    file.size_bytes / 1024
                ));
            }
        }

        report.push_str(
            "\n--------------------------------------------------------------------------------\n",
        );
        report.push_str("                              MODEL DETAILS\n");
        report.push_str(
            "--------------------------------------------------------------------------------\n\n",
        );

        report.push_str(&format!("Name: {}\n", model.name));
        report.push_str(&format!("Node Count: {}\n", model.node_count));
        report.push_str(&format!("Meshes: {}\n", model.meshes.len()));
        report.push_str(&format!("Skins: {}\n", model.skins.len()));
        report.push_str(&format!("Textures: {}\n\n", model.textures.len()));

        report.push_str("Mesh Details:\n");
        for (name, mesh) in &model.meshes {
            report.push_str(&format!("  {}:\n", name));
            report.push_str(&format!("    Vertices: {}\n", mesh.vertices.len()));
            report.push_str(&format!(
                "    Indices: {} ({} triangles)\n",
                mesh.indices.len(),
                mesh.indices.len() / 3
            ));
            if let Some(ref skin_data) = mesh.skin_data {
                report.push_str(&format!(
                    "    Skinned: Yes ({} skinned vertices)\n",
                    skin_data.skinned_vertices.len()
                ));
                if let Some(skin_idx) = skin_data.skin_index {
                    report.push_str(&format!("    Skin Index: {}\n", skin_idx));
                }
            } else {
                report.push_str("    Skinned: No\n");
            }
        }

        report.push_str("\nSkin Details:\n");
        for (idx, skin) in model.skins.iter().enumerate() {
            report.push_str(&format!("  Skin {}:\n", idx));
            report.push_str(&format!("    Name: {:?}\n", skin.name));
            report.push_str(&format!("    Joints: {}\n", skin.joints.len()));
            report.push_str(&format!(
                "    Inverse Bind Matrices: {}\n",
                skin.inverse_bind_matrices.len()
            ));
        }

        report.push_str(
            "\n--------------------------------------------------------------------------------\n",
        );
        report.push_str("                           PREFAB HIERARCHY\n");
        report.push_str(
            "--------------------------------------------------------------------------------\n\n",
        );

        fn write_node_tree(node: &PrefabNode, indent: usize, report: &mut String) {
            let prefix = "  ".repeat(indent);
            let name = node
                .components
                .name
                .as_ref()
                .map_or("<unnamed>", |n| n.0.as_str());
            let node_idx = node.node_index.map_or("N/A".to_string(), |i| i.to_string());
            report.push_str(&format!("{}[{}] {}\n", prefix, node_idx, name));

            let t = &node.local_transform.translation;
            let r = &node.local_transform.rotation;
            let s = &node.local_transform.scale;
            report.push_str(&format!(
                "{}  T: ({:.3}, {:.3}, {:.3})\n",
                prefix, t.x, t.y, t.z
            ));
            report.push_str(&format!(
                "{}  R: ({:.3}, {:.3}, {:.3}, {:.3})\n",
                prefix, r.i, r.j, r.k, r.w
            ));
            report.push_str(&format!(
                "{}  S: ({:.3}, {:.3}, {:.3})\n",
                prefix, s.x, s.y, s.z
            ));

            for child in &node.children {
                write_node_tree(child, indent + 1, report);
            }
        }

        for root in &model.prefab.root_nodes {
            write_node_tree(root, 0, &mut report);
        }

        report.push_str(
            "\n--------------------------------------------------------------------------------\n",
        );
        report.push_str("                           ANIMATION DETAILS\n");
        report.push_str(
            "--------------------------------------------------------------------------------\n\n",
        );

        report.push_str(&format!(
            "Total Animations Included: {}\n\n",
            animations.len()
        ));

        for anim in animations {
            report.push_str(&format!("Animation: {}\n", anim.name));
            report.push_str(&format!("  Duration: {:.3}s\n", anim.duration));
            report.push_str(&format!("  Channels: {}\n", anim.channels.len()));

            let mut channel_breakdown: HashMap<&str, usize> = HashMap::new();
            for channel in &anim.channels {
                let prop = match channel.target_property {
                    AnimationProperty::Translation => "Translation",
                    AnimationProperty::Rotation => "Rotation",
                    AnimationProperty::Scale => "Scale",
                    _ => "Other",
                };
                *channel_breakdown.entry(prop).or_insert(0) += 1;
            }
            for (prop, count) in &channel_breakdown {
                report.push_str(&format!("    {}: {} channels\n", prop, count));
            }
            report.push_str("\n");
        }

        report.push_str(
            "--------------------------------------------------------------------------------\n",
        );
        report.push_str("                              CONSOLE LOG\n");
        report.push_str(
            "--------------------------------------------------------------------------------\n\n",
        );

        for line in &self.report_lines {
            report.push_str(line);
            report.push_str("\n");
        }

        report.push_str(
            "\n================================================================================\n",
        );
        report.push_str("                              END OF REPORT\n");
        report.push_str(
            "================================================================================\n",
        );

        std::fs::write(&report_path, &report)?;
        println!("Report written to: {}", report_path.display());

        Ok(())
    }
}

#[derive(Debug, Clone)]
struct FbxFileInfo {
    path: PathBuf,
    name: String,
    size_bytes: u64,
    is_model: bool,
    selected: bool,
}

#[derive(Debug, Clone)]
struct LogEntry {
    timestamp: String,
    message: String,
}

#[derive(Default, PartialEq)]
enum AppState {
    #[default]
    Idle,
    Importing,
    Ready,
    Exporting,
}

struct LoadedModel {
    name: String,
    prefab: Prefab,
    skins: Vec<GltfSkin>,
    meshes: HashMap<String, nightshade::ecs::mesh::Mesh>,
    textures: HashMap<String, (Vec<u8>, u32, u32)>,
    node_count: usize,
}

struct LoadedAnimation {
    name: String,
    clips: Vec<AnimationClip>,
}

struct MixamoConverter {
    app_state: AppState,
    console_log: Vec<LogEntry>,
    status_text: String,
    fbx_files: Vec<FbxFileInfo>,
    selected_model_index: Option<usize>,
    temp_dir: Option<PathBuf>,
    loaded_models: Vec<LoadedModel>,
    loaded_animations: Vec<LoadedAnimation>,
    preview_entity: Option<Entity>,
    strip_root_motion: bool,
    exported_glb_path: Option<PathBuf>,
    needs_preview_spawn: bool,
}

impl Default for MixamoConverter {
    fn default() -> Self {
        Self {
            app_state: AppState::Idle,
            console_log: Vec::new(),
            status_text: String::from("Ready. Drop a folder or zip file, or click Import."),
            fbx_files: Vec::new(),
            selected_model_index: None,
            temp_dir: None,
            loaded_models: Vec::new(),
            loaded_animations: Vec::new(),
            preview_entity: None,
            strip_root_motion: true,
            exported_glb_path: None,
            needs_preview_spawn: false,
        }
    }
}

impl MixamoConverter {
    fn log(&mut self, message: &str) {
        let timestamp = Local::now().format("%H:%M:%S").to_string();
        self.console_log.push(LogEntry {
            timestamp,
            message: message.to_string(),
        });
        if self.console_log.len() > 100 {
            self.console_log.remove(0);
        }
    }

    fn set_status(&mut self, status: &str) {
        self.status_text = status.to_string();
        self.log(status);
    }

    fn import_path(&mut self, path: &Path) {
        self.app_state = AppState::Importing;
        self.fbx_files.clear();
        self.loaded_models.clear();
        self.loaded_animations.clear();
        self.selected_model_index = None;

        let source_path = if path.extension().is_some_and(|e| e == "zip") {
            self.set_status(&format!("Extracting zip: {}", path.display()));
            match self.extract_zip(path) {
                Ok(temp_path) => temp_path,
                Err(error) => {
                    self.set_status(&format!("Failed to extract zip: {}", error));
                    self.app_state = AppState::Idle;
                    return;
                }
            }
        } else {
            path.to_path_buf()
        };

        self.set_status(&format!(
            "Scanning for FBX files in: {}",
            source_path.display()
        ));
        self.scan_for_fbx_files(&source_path);

        if self.fbx_files.is_empty() {
            self.set_status("No FBX files found.");
            self.app_state = AppState::Idle;
            return;
        }

        self.set_status(&format!(
            "Found {} FBX files. Loading...",
            self.fbx_files.len()
        ));
        self.load_fbx_files();

        self.app_state = AppState::Ready;
        self.set_status(&format!(
            "Loaded {} models and {} animations. Select a model and animations to export.",
            self.loaded_models.len(),
            self.loaded_animations.len()
        ));

        if !self.loaded_models.is_empty() {
            self.needs_preview_spawn = true;
        }
    }

    fn extract_zip(&mut self, zip_path: &Path) -> Result<PathBuf, Box<dyn std::error::Error>> {
        let temp_dir = std::env::temp_dir().join(format!("mixamo_to_glb_{}", std::process::id()));
        std::fs::create_dir_all(&temp_dir)?;

        let file = std::fs::File::open(zip_path)?;
        let mut archive = zip::ZipArchive::new(file)?;

        for index in 0..archive.len() {
            let mut file = archive.by_index(index)?;
            let outpath = temp_dir.join(file.mangled_name());

            if file.is_dir() {
                std::fs::create_dir_all(&outpath)?;
            } else {
                if let Some(parent) = outpath.parent() {
                    std::fs::create_dir_all(parent)?;
                }
                let mut outfile = std::fs::File::create(&outpath)?;
                std::io::copy(&mut file, &mut outfile)?;
            }
        }

        self.temp_dir = Some(temp_dir.clone());
        self.log(&format!("Extracted to: {}", temp_dir.display()));
        Ok(temp_dir)
    }

    fn scan_for_fbx_files(&mut self, dir: &Path) {
        for entry in WalkDir::new(dir).into_iter().filter_map(|e| e.ok()) {
            let path = entry.path();
            if path
                .extension()
                .is_some_and(|e| e.eq_ignore_ascii_case("fbx"))
            {
                let metadata = std::fs::metadata(path).ok();
                let size_bytes = metadata.map_or(0, |m| m.len());
                let name = path
                    .file_stem()
                    .unwrap_or_default()
                    .to_string_lossy()
                    .to_string();

                let is_model = size_bytes > 1_000_000;

                self.fbx_files.push(FbxFileInfo {
                    path: path.to_path_buf(),
                    name,
                    size_bytes,
                    is_model,
                    selected: true,
                });

                self.log(&format!(
                    "Found: {} ({} KB) - {}",
                    path.file_name().unwrap_or_default().to_string_lossy(),
                    size_bytes / 1024,
                    if is_model { "MODEL" } else { "Animation" }
                ));
            }
        }

        self.fbx_files
            .sort_by(|a, b| b.size_bytes.cmp(&a.size_bytes));
    }

    fn load_fbx_files(&mut self) {
        let files: Vec<_> = self.fbx_files.clone();

        for file_info in &files {
            if file_info.is_model {
                self.log(&format!("Loading model: {}", file_info.name));
                match nightshade::ecs::prefab::import_fbx_from_path(&file_info.path) {
                    Ok(result) => {
                        if let Some(prefab) = result.prefabs.into_iter().next() {
                            self.loaded_models.push(LoadedModel {
                                name: file_info.name.clone(),
                                prefab,
                                skins: result.skins,
                                meshes: result.meshes,
                                textures: result.textures,
                                node_count: result.node_count,
                            });
                            self.log(&format!("Loaded model: {}", file_info.name));
                        }
                    }
                    Err(error) => {
                        self.log(&format!(
                            "Failed to load model {}: {}",
                            file_info.name, error
                        ));
                    }
                }
            } else {
                self.log(&format!("Loading animation: {}", file_info.name));
                match nightshade::ecs::prefab::import_fbx_animations_from_path(&file_info.path) {
                    Ok(clips) => {
                        if !clips.is_empty() {
                            self.loaded_animations.push(LoadedAnimation {
                                name: file_info.name.clone(),
                                clips,
                            });
                            self.log(&format!("Loaded animation: {}", file_info.name));
                        }
                    }
                    Err(error) => {
                        self.log(&format!(
                            "Failed to load animation {}: {}",
                            file_info.name, error
                        ));
                    }
                }
            }
        }

        if !self.loaded_models.is_empty() {
            self.selected_model_index = Some(0);
        }
    }

    fn export_glb(&mut self, output_path: &Path) {
        self.app_state = AppState::Exporting;
        self.set_status("Exporting to GLB...");

        let Some(model_index) = self.selected_model_index else {
            self.set_status("No model selected for export.");
            self.app_state = AppState::Ready;
            return;
        };

        let model_name = self.loaded_models[model_index].name.clone();

        let mut all_animations: Vec<AnimationClip> = Vec::new();
        for file_info in self.fbx_files.iter() {
            if !file_info.is_model
                && file_info.selected
                && let Some(loaded_anim) = self
                    .loaded_animations
                    .iter()
                    .find(|a| a.name == file_info.name)
            {
                for mut clip in loaded_anim.clips.clone() {
                    if self.strip_root_motion {
                        clip.channels.retain(|channel| {
                            channel.target_property != AnimationProperty::Translation
                        });
                    }
                    clip.name = file_info.name.clone();
                    all_animations.push(clip);
                }
            }
        }

        self.log(&format!(
            "Exporting model '{}' with {} animations",
            model_name,
            all_animations.len()
        ));

        let mut cli = CliConverter::new();
        cli.fbx_files = self.fbx_files.clone();

        let model_prefab = self.loaded_models[model_index].prefab.clone();
        let model_skins = self.loaded_models[model_index].skins.clone();
        let model_meshes = self.loaded_models[model_index].meshes.clone();
        let model_textures = self.loaded_models[model_index].textures.clone();
        let model_node_count = self.loaded_models[model_index].node_count;

        let model = LoadedModel {
            name: model_name.clone(),
            prefab: model_prefab,
            skins: model_skins,
            meshes: model_meshes,
            textures: model_textures,
            node_count: model_node_count,
        };

        match cli.build_glb_internal(&model, &all_animations) {
            Ok(glb_data) => {
                if let Err(e) = std::fs::write(output_path, &glb_data) {
                    self.set_status(&format!("Failed to write file: {}", e));
                } else {
                    self.set_status(&format!(
                        "Successfully exported {} bytes to: {}",
                        glb_data.len(),
                        output_path.display()
                    ));

                    if let Err(e) = cli.write_report(output_path, &model, &all_animations) {
                        self.log(&format!("Failed to write report: {}", e));
                    }

                    self.exported_glb_path = Some(output_path.to_path_buf());
                    self.log("GLB exported. Click 'Load Exported GLB' to verify in preview.");
                }
            }
            Err(error) => {
                self.set_status(&format!("Export failed: {}", error));
            }
        }

        self.app_state = AppState::Ready;
    }

    fn load_exported_glb(&mut self, world: &mut World) {
        let Some(glb_path) = self.exported_glb_path.clone() else {
            self.set_status("No exported GLB to load");
            return;
        };

        self.log(&format!("Loading exported GLB: {}", glb_path.display()));

        if let Some(entity) = self.preview_entity.take() {
            despawn_recursive_immediate(world, entity);
        }

        match import_gltf_from_path(&glb_path) {
            Ok(result) => {
                self.log(&format!(
                    "Loaded GLB: {} meshes, {} skins, {} animations",
                    result.meshes.len(),
                    result.skins.len(),
                    result.animations.len()
                ));

                for (name, mesh) in &result.meshes {
                    mesh_cache_insert(&mut world.resources.mesh_cache, name.clone(), mesh.clone());
                }

                for (name, (rgba_data, width, height)) in &result.textures {
                    world.queue_command(WorldCommand::LoadTexture {
                        name: name.clone(),
                        rgba_data: rgba_data.clone(),
                        width: *width,
                        height: *height,
                    });
                }

                if let Some(prefab) = result.prefabs.first() {
                    let entity = nightshade::ecs::prefab::spawn_prefab_with_skins(
                        world,
                        prefab,
                        &result.animations,
                        &result.skins,
                        Vec3::zeros(),
                    );

                    if let Some(player) = world.get_animation_player_mut(entity)
                        && !player.clips.is_empty()
                    {
                        player.play(0);
                        player.looping = true;
                        self.log(&format!("Playing animation 0 of {}", player.clips.len()));
                    }

                    self.preview_entity = Some(entity);
                    self.set_status(&format!(
                        "Loaded exported GLB with {} animations",
                        result.animations.len()
                    ));
                } else {
                    self.set_status("Failed: No prefabs in exported GLB");
                }
            }
            Err(error) => {
                self.set_status(&format!("Failed to load GLB: {}", error));
            }
        }
    }

    fn show_import_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("Zip files", &["zip"])
            .pick_file()
        {
            self.import_path(&path);
        }
    }

    fn show_folder_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new().pick_folder() {
            self.import_path(&path);
        }
    }

    fn show_export_dialog(&mut self) {
        if self.selected_model_index.is_none() {
            self.set_status("Please select a model first.");
            return;
        }

        let default_name = self
            .loaded_models
            .get(self.selected_model_index.unwrap())
            .map(|m| format!("{}.glb", m.name))
            .unwrap_or_else(|| "export.glb".to_string());

        if let Some(path) = rfd::FileDialog::new()
            .add_filter("GLB", &["glb"])
            .set_file_name(&default_name)
            .save_file()
        {
            self.export_glb(&path);
        }
    }

    fn spawn_preview(&mut self, world: &mut World) {
        if let Some(entity) = self.preview_entity.take() {
            despawn_recursive_immediate(world, entity);
        }

        let Some(model_index) = self.selected_model_index else {
            return;
        };

        let model = &self.loaded_models[model_index];

        for (name, mesh) in &model.meshes {
            mesh_cache_insert(&mut world.resources.mesh_cache, name.clone(), mesh.clone());
        }

        for (name, (rgba_data, width, height)) in &model.textures {
            world.queue_command(WorldCommand::LoadTexture {
                name: name.clone(),
                rgba_data: rgba_data.clone(),
                width: *width,
                height: *height,
            });
        }

        let mut all_animations: Vec<AnimationClip> = Vec::new();
        for file_info in &self.fbx_files {
            if !file_info.is_model
                && file_info.selected
                && let Some(loaded_anim) = self
                    .loaded_animations
                    .iter()
                    .find(|a| a.name == file_info.name)
            {
                for mut clip in loaded_anim.clips.clone() {
                    if self.strip_root_motion {
                        clip.channels.retain(|channel| {
                            channel.target_property != AnimationProperty::Translation
                        });
                    }
                    clip.name = file_info.name.clone();
                    all_animations.push(clip);
                }
            }
        }

        let entity = nightshade::ecs::prefab::spawn_prefab_with_skins(
            world,
            &model.prefab,
            &all_animations,
            &model.skins,
            Vec3::zeros(),
        );

        if let Some(transform) = world.get_local_transform_mut(entity) {
            transform.scale = Vec3::new(0.01, 0.01, 0.01);
        }
        world.mark_local_transform_dirty(entity);

        if let Some(player) = world.get_animation_player_mut(entity)
            && !player.clips.is_empty()
        {
            player.play(0);
            player.looping = true;
        }

        self.preview_entity = Some(entity);
        self.log(&format!("Spawned preview of model: {}", model.name));
    }
}

const SKY_HDR: &[u8] = include_bytes!("../../nightshade-examples/assets/sky/moonrise.hdr");

impl State for MixamoConverter {
    fn title(&self) -> &str {
        "Mixamo to GLB Converter"
    }

    fn initialize(&mut self, world: &mut World) {
        world.resources.user_interface.enabled = true;
        world.resources.graphics.show_grid = true;
        world.resources.graphics.atmosphere = Atmosphere::Hdr;

        load_hdr_skybox(world, SKY_HDR.to_vec());

        spawn_sun(world);

        let camera_entity = spawn_pan_orbit_camera(
            world,
            Vec3::new(0.0, 1.0, 0.0),
            5.0,
            0.0,
            std::f32::consts::FRAC_PI_6,
            "Main Camera".to_string(),
        );
        world.resources.active_camera = Some(camera_entity);

        self.log("Mixamo to GLB Converter initialized");
        self.log("Import a folder or zip file containing FBX files to begin");
    }

    fn ui(&mut self, world: &mut World, ui_context: &egui::Context) {
        egui::TopBottomPanel::top("top_panel").show(ui_context, |ui| {
            ui.horizontal(|ui| {
                ui.heading("Mixamo to GLB Converter");
                ui.separator();
                ui.label(&self.status_text);
            });
        });

        egui::SidePanel::left("control_panel")
            .min_width(350.0)
            .show(ui_context, |ui| {
                ui.heading("Import");
                ui.horizontal(|ui| {
                    if ui.button("Import Zip...").clicked() {
                        self.show_import_dialog();
                    }
                    if ui.button("Import Folder...").clicked() {
                        self.show_folder_dialog();
                    }
                });

                ui.separator();

                if self.app_state == AppState::Importing {
                    ui.horizontal(|ui| {
                        ui.spinner();
                        ui.label("Importing...");
                    });
                }

                if self.app_state == AppState::Exporting {
                    ui.horizontal(|ui| {
                        ui.spinner();
                        ui.label("Exporting...");
                    });
                }

                if !self.loaded_models.is_empty() {
                    ui.separator();
                    ui.heading("Models");

                    let mut model_changed = false;
                    for (index, model) in self.loaded_models.iter().enumerate() {
                        let selected = self.selected_model_index == Some(index);
                        if ui.selectable_label(selected, &model.name).clicked() {
                            self.selected_model_index = Some(index);
                            model_changed = true;
                        }
                    }

                    if model_changed {
                        self.spawn_preview(world);
                    }
                }

                if !self.loaded_animations.is_empty() {
                    ui.separator();
                    ui.heading("Animations");
                    ui.checkbox(&mut self.strip_root_motion, "Strip Root Motion");

                    ui.horizontal(|ui| {
                        if ui.button("Select All").clicked() {
                            for file_info in &mut self.fbx_files {
                                if !file_info.is_model {
                                    file_info.selected = true;
                                }
                            }
                        }
                        if ui.button("Select None").clicked() {
                            for file_info in &mut self.fbx_files {
                                if !file_info.is_model {
                                    file_info.selected = false;
                                }
                            }
                        }
                    });

                    egui::ScrollArea::vertical()
                        .max_height(300.0)
                        .show(ui, |ui| {
                            for file_info in &mut self.fbx_files {
                                if !file_info.is_model {
                                    ui.checkbox(&mut file_info.selected, &file_info.name);
                                }
                            }
                        });
                }

                ui.separator();

                let can_export =
                    self.selected_model_index.is_some() && self.app_state == AppState::Ready;

                ui.add_enabled_ui(can_export, |ui| {
                    if ui.button("Export GLB...").clicked() {
                        self.show_export_dialog();
                    }
                });

                if self.selected_model_index.is_some()
                    && self.app_state == AppState::Ready
                    && ui.button("Refresh Preview").clicked()
                {
                    self.spawn_preview(world);
                }

                if self.exported_glb_path.is_some()
                    && self.app_state == AppState::Ready
                    && ui.button("Load Exported GLB").clicked()
                {
                    self.load_exported_glb(world);
                }
            });

        egui::TopBottomPanel::bottom("console_panel")
            .min_height(120.0)
            .show(ui_context, |ui| {
                ui.heading("Console");
                egui::ScrollArea::vertical()
                    .stick_to_bottom(true)
                    .show(ui, |ui| {
                        for entry in &self.console_log {
                            ui.horizontal(|ui| {
                                ui.label(
                                    egui::RichText::new(&entry.timestamp)
                                        .color(egui::Color32::GRAY)
                                        .monospace(),
                                );
                                ui.label(&entry.message);
                            });
                        }
                    });
            });
    }

    fn run_systems(&mut self, world: &mut World) {
        if self.needs_preview_spawn {
            self.needs_preview_spawn = false;
            self.spawn_preview(world);
        }
        pan_orbit_camera_system(world);
    }

    fn on_keyboard_input(&mut self, world: &mut World, key_code: KeyCode, key_state: KeyState) {
        if matches!((key_code, key_state), (KeyCode::KeyQ, KeyState::Pressed)) {
            world.resources.window.should_exit = true;
        }
    }

    fn on_dropped_file(&mut self, _world: &mut World, path: &Path) {
        self.import_path(path);
    }
}
