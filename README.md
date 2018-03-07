# Basics
This project uses optical flow to show objects in motion. It uses the webcam to record movement and draws colored lines based on their directional speed. Additionally, this project uses color averaging to send messages to a granular synthesizer built in Pure Data.

# How do I run this repo?
This repo requires just a few Processing libraries (OpenCV, ProcessingVideo, OscP5) as well as MrPeach and [grambilib](https://github.com/rickygraham/grambilib) for Pure Data. Open the Pure Data patch and then run the Processing sketch. If all goes well, the console will tell you that you are connected to Port 8080 and you should be able to see sliders move within the Pure Data patch. This can all run on a laptop but for an installation, this should be run with a camera, projector, external sound card, and set of stereo speakers all connected to a Mac Mini.
# What is being tracked?
The Processing sketch uses OpenCV to capture the relative motion of objects. Audio is informed via the average RGB values from webcam input. 
# What is granular synthesis?
Think of how a tape machine reads audio. Magnetic tape runs across a playback head at a constant speed. Granular synthesis works in a similar way except we are constantly moving the playback head at different locations and speeds. Each split of audio is called a grain and only lasts for a couple of milliseconds. We can modify almost any characteristic of an audio grain. This type of synthesis allows for a sound which seemingly mimics biological systems through its complex evolution of simple premises. In this repo, we are only modifying the panning and the speed of the playback head but if you want to dive into the Pure Data patch, you can easily modify the pitch, jitter, grain size, and envelope of each grain individually.
# How can I change the audio?
This implementation currently uses 44.1k .wav files for audio. If you want a different audio file, you can replace the overwrite the old audio.wav with another 44.1k .wav file or you can follow the patch's comments into the *pd playback* object and follow the commented directions to change the two sample rate messages and/or the file being loaded. Pure Data will also read any file type's raw data as audio ;).