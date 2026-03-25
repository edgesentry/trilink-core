pub mod buffer;
pub mod bridge;
pub mod error;
pub mod ingress;

pub use error::TriError;

/// One complete detection event: image capture with pose, intrinsics, and inference results.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct FusionPacket {
    /// Image capture time (microseconds since UNIX epoch).
    pub capture_ts_us: u64,
    /// Platform pose at `capture_ts_us`, looked up from the pose buffer.
    pub pose: Transform4x4,
    /// Static camera calibration parameters.
    pub camera_k: CameraIntrinsics,
    /// Raw JPEG frame that was sent to the inference service.
    pub image_jpeg: Vec<u8>,
    /// Inference results with world coordinates resolved.
    pub detections: Vec<Detection>,
}

/// A single object detection with optional world-space location.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Detection {
    /// Class label returned by the inference service (e.g. `"person"`, `"crack"`, `"vehicle"`).
    pub class: String,
    pub confidence: f32,
    /// Pixel-space bounding box from the inference service.
    pub bbox: BBox2D,
    /// 3D world position after unprojection. `None` until resolved.
    pub world_pos: Option<Point3D>,
    /// Depth from ToF sensor if available.
    pub depth_m: Option<f32>,
}

/// Pinhole camera calibration parameters.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct CameraIntrinsics {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
}

impl CameraIntrinsics {
    /// Create intrinsics from a horizontal field of view, with the principal
    /// point at the image centre (`cx = width/2`, `cy = height/2`).
    ///
    /// Computes `fx = fy = (width/2) / tan(hfov_rad/2)`.
    ///
    /// # Example
    /// ```
    /// # use trilink_core::CameraIntrinsics;
    /// // 90° horizontal FOV on a 1920×1080 sensor
    /// let k = CameraIntrinsics::from_fov(1920, 1080, std::f64::consts::FRAC_PI_2);
    /// assert!((k.fx - 960.0).abs() < 1e-9);
    /// assert_eq!(k.cx, 960.0);
    /// assert_eq!(k.cy, 540.0);
    /// ```
    pub fn from_fov(width: u32, height: u32, hfov_rad: f64) -> Self {
        let fx = (width as f64 / 2.0) / (hfov_rad / 2.0).tan();
        Self { fx, fy: fx, cx: width as f64 / 2.0, cy: height as f64 / 2.0 }
    }

    /// Create intrinsics with explicit focal lengths and the principal point at
    /// the image centre (`cx = width/2`, `cy = height/2`).
    ///
    /// # Example
    /// ```
    /// # use trilink_core::CameraIntrinsics;
    /// let k = CameraIntrinsics::from_focal(1920, 1080, 800.0, 800.0);
    /// assert_eq!(k.fx, 800.0);
    /// assert_eq!(k.cx, 960.0);
    /// assert_eq!(k.cy, 540.0);
    /// ```
    pub fn from_focal(width: u32, height: u32, fx: f64, fy: f64) -> Self {
        Self { fx, fy, cx: width as f64 / 2.0, cy: height as f64 / 2.0 }
    }
}

/// Homogeneous 4×4 transform (platform pose in world frame).
///
/// Wraps a [`glam::Mat4`] directly for zero-overhead SIMD arithmetic, inversion,
/// and operator overloading. Serialized as a row-major `{"matrix": [f32; 16]}`
/// JSON object for human readability.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform4x4 {
    /// Underlying column-major matrix (glam convention).
    pub mat: glam::Mat4,
}

impl Transform4x4 {
    /// Identity transform — platform at origin, no rotation.
    pub fn identity() -> Self {
        Self { mat: glam::Mat4::IDENTITY }
    }

    /// Construct from a **row-major** flat array (the human-readable layout).
    ///
    /// ```text
    /// arr = [r00, r01, r02, r03,   // row 0
    ///        r10, r11, r12, r13,   // row 1
    ///        r20, r21, r22, r23,   // row 2
    ///        r30, r31, r32, r33]   // row 3
    /// ```
    pub fn from_row_major(arr: [f32; 16]) -> Self {
        // from_cols_array treats input as column-major; transpose converts row→col.
        Self { mat: glam::Mat4::from_cols_array(&arr).transpose() }
    }

    /// Return a **row-major** flat array representation.
    ///
    /// Inverse of [`from_row_major`](Transform4x4::from_row_major).
    pub fn to_row_major(&self) -> [f32; 16] {
        self.mat.transpose().to_cols_array()
    }

    /// Apply this transform to a camera-space point, returning a world-space [`Point3D`].
    ///
    /// Computes `P_world = self · [xc, yc, zc, 1]ᵀ` via [`glam::Mat4::transform_point3`].
    pub fn transform_point(&self, xc: f32, yc: f32, zc: f32) -> Point3D {
        self.mat.transform_point3(glam::vec3(xc, yc, zc)).into()
    }
}

impl From<glam::Mat4> for Transform4x4 {
    fn from(mat: glam::Mat4) -> Self {
        Self { mat }
    }
}

impl Default for Transform4x4 {
    fn default() -> Self {
        Self::identity()
    }
}

impl std::ops::Mul for Transform4x4 {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Self { mat: self.mat * rhs.mat }
    }
}

impl serde::Serialize for Transform4x4 {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        use serde::ser::SerializeStruct;
        let mut st = s.serialize_struct("Transform4x4", 1)?;
        st.serialize_field("matrix", &self.to_row_major())?;
        st.end()
    }
}

impl<'de> serde::Deserialize<'de> for Transform4x4 {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        #[derive(serde::Deserialize)]
        struct Helper {
            matrix: [f32; 16],
        }
        let h = Helper::deserialize(d)?;
        Ok(Transform4x4::from_row_major(h.matrix))
    }
}

/// 3D world-space point, expressed in a **local coordinate frame**.
///
/// # Coordinate frame requirement
///
/// All coordinates must be local offsets from a nearby origin (e.g. the SLAM
/// map origin or the scanner start position), **not** absolute geodetic values
/// such as UTM easting/northing. Keeping coordinates local ensures that `f32`
/// precision is adequate.
///
/// # Precision analysis
///
/// `f32` provides ~7 significant decimal digits (machine epsilon ≈ 1.2 × 10⁻⁷).
/// At a coordinate magnitude of `M` metres, the representable step size is
/// approximately `M × 1.2 × 10⁻⁷` metres:
///
/// | Max coordinate magnitude | Representable step | Adequate for 1 cm target? |
/// |---|---|---|
/// | 100 m (large building) | ~0.012 mm | Yes |
/// | 1 km (city block) | ~0.12 mm | Yes |
/// | 10 km | ~1.2 mm | Yes |
/// | 100 km (UTM scale) | ~12 mm | **No** |
///
/// The system targets 1 cm accuracy over construction-site distances (≤ 1 km).
/// `f32` is sufficient for this range. If absolute geodetic coordinates are
/// ever required, convert to a local tangent-plane frame first.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct Point3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl From<Point3D> for glam::Vec3 {
    fn from(p: Point3D) -> Self {
        glam::vec3(p.x, p.y, p.z)
    }
}

impl From<glam::Vec3> for Point3D {
    fn from(v: glam::Vec3) -> Self {
        Point3D { x: v.x, y: v.y, z: v.z }
    }
}

/// Pixel-space bounding box (inclusive min, exclusive max convention).
///
/// Fields are `f32` to accept both integer pixel coords and sub-pixel float
/// coords returned by most inference APIs (e.g. YOLO-style services).
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct BBox2D {
    pub u0: f32,
    pub v0: f32,
    pub u1: f32,
    pub v1: f32,
}

impl BBox2D {
    /// Returns the center pixel `(u, v)`.
    pub fn center(&self) -> (f64, f64) {
        (
            (self.u0 + self.u1) as f64 / 2.0,
            (self.v0 + self.v1) as f64 / 2.0,
        )
    }
}

/// One LiDAR/ToF sweep: world-space points with optional per-point intensity.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct PointCloud {
    /// Sweep capture time (microseconds since UNIX epoch).
    pub capture_ts_us: u64,
    /// World-space 3D points in the sweep.
    pub points: Vec<Point3D>,
    /// Per-point LiDAR intensity, parallel to `points`. `None` if not available.
    pub intensities: Option<Vec<f32>>,
}

/// Per-pixel depth map in metres; output of `project_to_depth_map`.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct DepthMap {
    pub width: u32,
    pub height: u32,
    /// Row-major depth values in metres. `f32::INFINITY` means no point projected here.
    pub data: Vec<f32>,
}

impl DepthMap {
    /// Returns the depth at pixel `(u, v)`. Panics if out of bounds.
    pub fn get(&self, u: u32, v: u32) -> f32 {
        self.data[(v * self.width + u) as usize]
    }

    /// Returns `true` if pixel `(u, v)` has a finite depth value.
    pub fn has_depth(&self, u: u32, v: u32) -> bool {
        self.get(u, v).is_finite()
    }
}

/// Per-cell max height map in metres; output of `project_to_height_map`.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct HeightMap {
    /// World-space X coordinate of the map origin (top-left corner).
    pub origin_x: f32,
    /// World-space Y coordinate of the map origin (top-left corner).
    pub origin_y: f32,
    /// Cell size in metres.
    pub resolution_m: f32,
    pub width: u32,
    pub height: u32,
    /// Row-major max height values in metres. `f32::NAN` means no data in this cell.
    ///
    /// Serialized as JSON `null` for NaN cells (JSON has no NaN literal).
    #[serde(with = "nan_as_null_vec")]
    pub data: Vec<f32>,
}

/// Serde helper: serializes a `Vec<f32>` where `NaN` values round-trip as JSON `null`.
mod nan_as_null_vec {
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    pub fn serialize<S: Serializer>(data: &[f32], s: S) -> Result<S::Ok, S::Error> {
        let opts: Vec<Option<f32>> = data.iter().map(|&v| if v.is_nan() { None } else { Some(v) }).collect();
        opts.serialize(s)
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<Vec<f32>, D::Error> {
        let opts: Vec<Option<f32>> = Vec::deserialize(d)?;
        Ok(opts.into_iter().map(|v| v.unwrap_or(f32::NAN)).collect())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- CameraIntrinsics constructors ---

    #[test]
    fn from_fov_90_degrees_1920x1080() {
        // 90° hfov → tan(45°) = 1 → fx = width/2 = 960
        let k = CameraIntrinsics::from_fov(1920, 1080, std::f64::consts::FRAC_PI_2);
        assert!((k.fx - 960.0).abs() < 1e-9, "fx should equal width/2 for 90° FOV");
        assert!((k.fy - 960.0).abs() < 1e-9, "fy should equal fx");
        assert_eq!(k.cx, 960.0);
        assert_eq!(k.cy, 540.0);
    }

    #[test]
    fn from_fov_60_degrees_640x480() {
        // 60° hfov → tan(30°) = 1/√3 → fx = 320 * √3 ≈ 554.256
        let k = CameraIntrinsics::from_fov(640, 480, std::f64::consts::PI / 3.0);
        let expected_fx = 320.0 / (std::f64::consts::PI / 6.0).tan();
        assert!((k.fx - expected_fx).abs() < 1e-9);
        assert_eq!(k.cx, 320.0);
        assert_eq!(k.cy, 240.0);
    }

    #[test]
    fn from_focal_sets_fields_and_centres_principal_point() {
        let k = CameraIntrinsics::from_focal(1920, 1080, 800.0, 600.0);
        assert_eq!(k.fx, 800.0);
        assert_eq!(k.fy, 600.0);
        assert_eq!(k.cx, 960.0);
        assert_eq!(k.cy, 540.0);
    }

    #[test]
    fn from_focal_square_sensor() {
        let k = CameraIntrinsics::from_focal(512, 512, 400.0, 400.0);
        assert_eq!(k.cx, 256.0);
        assert_eq!(k.cy, 256.0);
    }

    // --- BBox2D::center ---

    #[test]
    fn bbox_center_symmetric() {
        let bbox = BBox2D { u0: 10.0, v0: 20.0, u1: 30.0, v1: 40.0 };
        assert_eq!(bbox.center(), (20.0, 30.0));
    }

    #[test]
    fn bbox_center_fractional() {
        // Odd-width box: centre falls on a half-pixel
        let bbox = BBox2D { u0: 0.0, v0: 0.0, u1: 1.0, v1: 3.0 };
        assert_eq!(bbox.center(), (0.5, 1.5));
    }

    #[test]
    fn bbox_center_degenerate_zero_size() {
        let bbox = BBox2D { u0: 5.0, v0: 7.0, u1: 5.0, v1: 7.0 };
        assert_eq!(bbox.center(), (5.0, 7.0));
    }

    // --- Transform4x4::transform_point ---

    #[test]
    fn transform_point_identity_returns_input() {
        let p = Transform4x4::identity().transform_point(1.0, 2.0, 3.0);
        assert!((p.x - 1.0).abs() < 1e-6);
        assert!((p.y - 2.0).abs() < 1e-6);
        assert!((p.z - 3.0).abs() < 1e-6);
    }

    #[test]
    fn transform_point_translation_offsets_correctly() {
        // Translation-only matrix: identity rotation, tx=10, ty=20, tz=30
        #[rustfmt::skip]
        let t = Transform4x4::from_row_major([
            1.0, 0.0, 0.0, 10.0,
            0.0, 1.0, 0.0, 20.0,
            0.0, 0.0, 1.0, 30.0,
            0.0, 0.0, 0.0,  1.0,
        ]);
        let p = t.transform_point(1.0, 2.0, 3.0);
        assert!((p.x - 11.0).abs() < 1e-5);
        assert!((p.y - 22.0).abs() < 1e-5);
        assert!((p.z - 33.0).abs() < 1e-5);
    }

    // --- Transform4x4::identity ---

    #[test]
    fn identity_diagonal_is_one() {
        let m = Transform4x4::identity().to_row_major();
        assert_eq!(m[0],  1.0); // [0,0]
        assert_eq!(m[5],  1.0); // [1,1]
        assert_eq!(m[10], 1.0); // [2,2]
        assert_eq!(m[15], 1.0); // [3,3]
    }

    #[test]
    fn identity_off_diagonal_is_zero() {
        let m = Transform4x4::identity().to_row_major();
        for (i, &v) in m.iter().enumerate() {
            let row = i / 4;
            let col = i % 4;
            if row != col {
                assert_eq!(v, 0.0, "matrix[{i}] (row={row}, col={col}) expected 0");
            }
        }
    }

    // --- Serde roundtrips ---

    #[test]
    fn fusion_packet_roundtrip() {
        let packet = FusionPacket {
            capture_ts_us: 42_000_000,
            pose: Transform4x4::identity(),
            camera_k: CameraIntrinsics { fx: 800.0, fy: 800.0, cx: 960.0, cy: 540.0 },
            image_jpeg: vec![0xff, 0xd8, 0xff, 0xd9],
            detections: vec![Detection {
                class: "scratch".to_string(),
                confidence: 0.92,
                bbox: BBox2D { u0: 100.0, v0: 200.0, u1: 150.0, v1: 250.0 },
                world_pos: Some(Point3D { x: 1.0, y: 2.0, z: 3.0 }),
                depth_m: Some(2.5),
            }],
        };
        let json = serde_json::to_string(&packet).unwrap();
        let decoded: FusionPacket = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.capture_ts_us, 42_000_000);
        assert_eq!(decoded.detections.len(), 1);
        assert_eq!(decoded.detections[0].class, "scratch");
        assert!((decoded.detections[0].confidence - 0.92).abs() < 1e-5);
        let wp = decoded.detections[0].world_pos.unwrap();
        assert_eq!(wp.x, 1.0);
        assert_eq!(wp.y, 2.0);
        assert_eq!(wp.z, 3.0);
    }

    #[test]
    fn detection_with_no_world_pos_roundtrip() {
        let d = Detection {
            class: "dent".to_string(),
            confidence: 0.75,
            bbox: BBox2D { u0: 0.0, v0: 0.0, u1: 10.0, v1: 10.0 },
            world_pos: None,
            depth_m: None,
        };
        let json = serde_json::to_string(&d).unwrap();
        let decoded: Detection = serde_json::from_str(&json).unwrap();
        assert!(decoded.world_pos.is_none());
        assert!(decoded.depth_m.is_none());
        assert_eq!(decoded.class, "dent");
    }

    // --- PointCloud ---

    #[test]
    fn point_cloud_roundtrip() {
        let pc = PointCloud {
            capture_ts_us: 1_000_000,
            points: vec![Point3D { x: 1.0, y: 2.0, z: 3.0 }],
            intensities: Some(vec![0.8]),
        };
        let json = serde_json::to_string(&pc).unwrap();
        let decoded: PointCloud = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.capture_ts_us, 1_000_000);
        assert_eq!(decoded.points.len(), 1);
        assert!((decoded.points[0].x - 1.0).abs() < 1e-6);
        assert_eq!(decoded.intensities.as_ref().unwrap().len(), 1);
        assert!((decoded.intensities.unwrap()[0] - 0.8).abs() < 1e-6);
    }

    #[test]
    fn point_cloud_no_intensities_roundtrip() {
        let pc = PointCloud {
            capture_ts_us: 0,
            points: vec![],
            intensities: None,
        };
        let json = serde_json::to_string(&pc).unwrap();
        let decoded: PointCloud = serde_json::from_str(&json).unwrap();
        assert!(decoded.intensities.is_none());
        assert!(decoded.points.is_empty());
    }

    // --- DepthMap ---

    #[test]
    fn depth_map_get_and_has_depth() {
        // 2×2 depth map: top-left has depth 1.5, rest is infinity
        let dm = DepthMap {
            width: 2,
            height: 2,
            data: vec![1.5, f32::INFINITY, f32::INFINITY, f32::INFINITY],
        };
        assert_eq!(dm.get(0, 0), 1.5);
        assert!(dm.has_depth(0, 0));
        assert_eq!(dm.get(1, 0), f32::INFINITY);
        assert!(!dm.has_depth(1, 0));
    }

    #[test]
    fn depth_map_roundtrip() {
        let dm = DepthMap {
            width: 1,
            height: 1,
            data: vec![2.0],
        };
        let json = serde_json::to_string(&dm).unwrap();
        let decoded: DepthMap = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.width, 1);
        assert_eq!(decoded.height, 1);
        assert!((decoded.data[0] - 2.0).abs() < 1e-6);
    }

    // --- HeightMap ---

    #[test]
    fn height_map_roundtrip() {
        let hm = HeightMap {
            origin_x: 0.0,
            origin_y: 0.0,
            resolution_m: 0.1,
            width: 2,
            height: 2,
            data: vec![1.0, f32::NAN, 0.5, f32::NAN],
        };
        let json = serde_json::to_string(&hm).unwrap();
        let decoded: HeightMap = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.width, 2);
        assert_eq!(decoded.height, 2);
        assert!((decoded.resolution_m - 0.1).abs() < 1e-6);
        assert!((decoded.data[0] - 1.0).abs() < 1e-6);
        assert!(decoded.data[1].is_nan());
        assert!((decoded.data[2] - 0.5).abs() < 1e-6);
    }
}
