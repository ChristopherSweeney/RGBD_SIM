
avconv -framerate 30 -y -i $1/%05d_input_depth.png -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p $1_input_depth.mp4
avconv -framerate 30 -y -i $1/%05d_masked_depth.png -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p $1_masked_depth.mp4
avconv -framerate 30 -y -i $1/%05d_mask.png -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p $1_mask.mp4
avconv -framerate 30 -y -i $1/%05d_rgb.png -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p $1_rgb.mp4
rm -fr /tmp/image_tmp
mkdir /tmp/image_tmp
for filename in $1/*_rgb.png; do
	name=${filename##*/}
	base=${name%_rgb.png}
	echo $filename
	composite -blend 30 $1/${base}_rgb.png $1/${base}_masked_depth.png /tmp/image_tmp/${base}_rgb_plus_masked_depth.png
	composite -blend 30 $1/${base}_rgb.png $1/${base}_mask.png /tmp/image_tmp/${base}_rgb_plus_mask.png
	composite -blend 30 $1/${base}_rgb.png $1/${base}_input_depth.png /tmp/image_tmp/${base}_rgb_plus_input_depth.png
done
avconv -framerate 30 -y -i /tmp/image_tmp/%05d_rgb_plus_masked_depth.png -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p $1_rgb_plus_masked_depth.mp4
avconv -framerate 30 -y -i /tmp/image_tmp/%05d_rgb_plus_mask.png -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p $1_rgb_plus_mask.mp4
avconv -framerate 30 -y -i /tmp/image_tmp/%05d_rgb_plus_input_depth.png -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p $1_rgb_plus_input_depth.mp4
