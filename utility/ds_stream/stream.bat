start cmd /k ..\..\..\gstreamer\1.0\x86\bin\gst-launch-1.0 udpsrc -e port=5804 ! "application/x-rtp, payload=127" ! tee name=t ! queue ! rtph264depay ! h264parse ! mp4mux ! filesink location=output.mp4 t. ! queue ! rtph264depay ! avdec_h264 ! gdkpixbufoverlay location=C:/Users/ubuntu/Pictures/Overlays/overlay.png offset-x=0 offset-y=0 ! fpsdisplaysink sync=false text-overlay=false