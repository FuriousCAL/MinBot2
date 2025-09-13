# MinBot2 Vision System Testing Guide

## 🚀 Step-by-Step Testing Instructions

Follow these steps to test your new PhotonVision integration:

---

## Step 1: Basic System Startup

### 1.1 Start Robot Simulation
```bash
# In your MinBot2 directory
./gradlew simulateJava

# Or if you prefer the VS Code way:
# Press Ctrl+Shift+P → "WPILib: Simulate Robot Code"
```

### 1.2 Launch Driver Station
- Open FRC Driver Station
- Set to **Test Mode** (safest for initial testing)
- Click **Enable**

### 1.3 Open SmartDashboard
- Launch SmartDashboard or Glass
- You should see the robot simulator window open

---

## Step 2: Vision System Health Check

### 2.1 Check Camera Connection Status
Look for these keys on SmartDashboard:

**✅ Good Signs:**
- `Vision/Connected`: `true`
- `Vision/Frame Count`: Should be increasing (shows camera is sending data)
- `Vision/Consecutive Failures`: Should be `0` or low

**❌ Problem Signs:**
- `Vision/Connected`: `false`
- `Vision/Consecutive Failures`: High number
- `Vision/Frame Count`: Not increasing

### 2.2 Verify Camera Performance
**Target Values:**
- `Vision/Average Latency (ms)`: Should be < 50ms (your camera was at 24ms - excellent!)
- `Vision/Frame Count`: Should steadily increase every few seconds

---

## Step 3: AprilTag Detection Testing

### 3.1 Position AprilTags in Camera View
- Place some 2024 FRC AprilTags in front of your ArduCam1
- Make sure they're well-lit and clearly visible
- Distance: Start at 1-3 meters for best results

### 3.2 Monitor Target Detection
**SmartDashboard Keys to Watch:**
- `Vision/Has Targets`: Should show `true` when tags are visible
- `Vision/Target Count`: Number of tags the camera can see
- `Vision/Best Target ID`: ID of the clearest tag (should match your physical tags)

**Expected Behavior:**
```
No tags visible:
Vision/Has Targets: false
Vision/Target Count: 0

Tags visible:
Vision/Has Targets: true  
Vision/Target Count: 1, 2, 3, etc.
Vision/Best Target ID: [tag number]
```

---

## Step 4: Pose Estimation Testing

### 4.1 Single Tag Test
- Show **one** AprilTag to camera
- Check SmartDashboard:

```
Vision/Tags Used: 1
Vision/Estimated Pose: (x.xx, y.yy, angle°)
Vision/Best Target Ambiguity: < 0.3 (lower is better)
```

### 4.2 Multi-Tag Test (Best Accuracy)
- Show **multiple** AprilTags simultaneously
- You should see:

```
Vision/Tags Used: 2 or 3+ (better!)
Vision/Estimated Pose: (more accurate position)
Vision/Best Target Ambiguity: < 0.2 (even better)
```

### 4.3 Watch Robot on Field2d
- The robot position on the Field2d widget should update based on AprilTag detections
- Move tags around - robot position should change accordingly

---

## Step 5: System Integration Test

### 5.1 Enable Robot
- Switch Driver Station to **TeleOperated Mode**
- **Enable** the robot
- Use your Xbox controller to drive around

### 5.2 Watch Pose Updates
- As you drive around with AprilTags visible, the vision system should:
  - Update robot position continuously
  - Show smooth movement on Field2d widget
  - Maintain connection (`Vision/Connected: true`)

---

## 🔧 Troubleshooting Guide

### Problem: `Vision/Connected: false`

**Check:**
1. **Network Connection:**
   ```bash
   ping 192.168.86.30
   # Should respond successfully
   ```

2. **PhotonVision Dashboard:**
   - Open http://192.168.86.30:5800/#/dashboard
   - Verify "ArduCam1" is active
   - Check that "Pipe8x6" pipeline is running

3. **Camera Name:**
   - Verify `Constants.java` has `PRIMARY_CAMERA_NAME = "ArduCam1"`
   - Must match exactly what's in PhotonVision dashboard

### Problem: `Vision/Has Targets: false` (tags not detected)

**Check:**
1. **AprilTag Setup:**
   - Using 2024 FRC AprilTags? (not 2023 or older)
   - Tags well-lit and clearly visible?
   - Distance: 1-5 meters works best

2. **Camera Pipeline:**
   - In PhotonVision dashboard, switch to different pipeline if needed
   - Verify AprilTag detection is enabled in pipeline

3. **Camera Focus:**
   - Is ArduCam1 properly focused?
   - Check camera stream in PhotonVision dashboard

### Problem: Poor Pose Estimates

**Tuning Options in `Constants.java`:**
```java
// Make vision less strict (accepts lower quality)
public static final double MAX_POSE_AMBIGUITY = 0.5; // was 0.3

// Reduce confidence in vision measurements
public static final double[] VISION_MEASUREMENT_STDDEVS = {1.0, 1.0, Math.toRadians(45)};
```

---

## 📊 Performance Benchmarks

### Excellent Performance:
- `Vision/Connected`: `true` consistently
- `Vision/Average Latency (ms)`: < 30ms
- `Vision/Consecutive Failures`: 0-2
- `Vision/Best Target Ambiguity`: < 0.2 with multiple tags

### Good Performance:
- `Vision/Average Latency (ms)`: < 50ms  
- `Vision/Consecutive Failures`: < 5
- `Vision/Best Target Ambiguity`: < 0.3

### Needs Improvement:
- `Vision/Average Latency (ms)`: > 100ms
- `Vision/Consecutive Failures`: > 10
- Frequent disconnections

---

## 🎯 Test Scenarios to Try

### Scenario 1: Basic Detection
1. Place one AprilTag 2 meters from camera
2. Verify detection and pose estimation
3. Move tag left/right - pose should update

### Scenario 2: Multi-Tag Accuracy
1. Place 2-3 AprilTags in camera view
2. Compare pose accuracy vs single tag
3. Should see lower ambiguity values

### Scenario 3: Distance Testing
1. Start close (1m) and move tags further away (up to 5m)
2. Monitor at what distance detection becomes unreliable
3. Adjust `MAX_APRILTAG_DISTANCE_METERS` if needed

### Scenario 4: Movement Testing
1. Enable robot and drive around
2. Keep AprilTags visible while moving
3. Verify pose updates smoothly during motion

---

## 📝 What to Log During Testing

Keep notes on:
- **Connection stability**: Any disconnections?
- **Detection range**: Max reliable distance?
- **Performance**: Average latency values?
- **Accuracy**: How well do pose estimates match expected positions?
- **Lighting conditions**: What lighting works best?

---

## ✅ Success Criteria

Your vision system is working well when:

1. **Consistent Connection**: `Vision/Connected` stays `true`
2. **Reliable Detection**: Tags detected at 3+ meter range
3. **Good Performance**: Latency < 50ms, minimal failures  
4. **Accurate Poses**: Robot position matches expected location
5. **Smooth Integration**: Vision updates don't interfere with driving

---

## 🚀 Next Steps After Successful Testing

Once basic testing passes:
1. **Calibrate Camera Transform**: Fine-tune robot-to-camera positioning
2. **Tune Standard Deviations**: Optimize vision vs odometry weighting  
3. **Test Edge Cases**: Poor lighting, distant tags, partial occlusion
4. **Phase 2 Development**: Auto-alignment commands, vision-assisted autonomous

---

**Happy Testing! 🤖📷**
