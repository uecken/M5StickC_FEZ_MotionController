# M5StickC_FEZ_MotionController

## How to
 - This controller has 3 mode. You can change the mode by press M5-Button for 1 second.
   - Fantaze Earth Zero
   - Fortnite (some FPS)
   - Mouse
 - Roll/Pitch/Yaw are used to recognize XY cordinates and skill-motion by Mahony-filter.
 
## [Youtube](https://www.youtube.com/watch?v=hiMvGo1NXqg)

## Note
 - Should change sampleFreq from `#define sampleFreq	150.0f` to  `#define sampleFreq	150.0f` in ...\Arduino\libraries\M5StickC\src\utility\MahonyAHRS.cpp, then compile.
