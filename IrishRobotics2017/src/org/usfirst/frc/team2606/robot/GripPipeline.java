package org.usfirst.frc.team2606.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.vision.VisionPipeline;

import org.opencv.core.*;
import org.opencv.imgproc.*;

/**
* GripPipeline class.
*
* <p>An OpenCV pipeline generated by GRIP.
*
* @author GRIP
*/
public class GripPipeline implements VisionPipeline {

	//Outputs
	private Mat cvResizeOutput = new Mat();
	private Mat blurOutput = new Mat();
	private Mat rgbThreshold0Output = new Mat();
	private Mat hsvThresholdOutput = new Mat();
	private Mat hslThreshold0Output = new Mat();
	private Mat cvBitwiseAnd0Output = new Mat();
	private Mat cvBitwiseAnd1Output = new Mat();
	private Mat rgbThreshold1Output = new Mat();
	private Mat hslThreshold1Output = new Mat();
	private Mat cvBitwiseAnd2Output = new Mat();
	private Mat cvSubtractOutput = new Mat();
	private Mat maskOutput = new Mat();
	private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
	private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();
	private ArrayList<MatOfPoint> convexHullsOutput = new ArrayList<MatOfPoint>();

	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	/**
	 * This is the primary method that runs the entire pipeline and updates the outputs.
	 */
	@Override	public void process(Mat source0) {
		// Step CV_resize0:
		Mat cvResizeSrc = source0;
		Size cvResizeDsize = new Size(0, 0);
		double cvResizeFx = 0.25;
		double cvResizeFy = 0.25;
		int cvResizeInterpolation = Imgproc.INTER_LINEAR;
		cvResize(cvResizeSrc, cvResizeDsize, cvResizeFx, cvResizeFy, cvResizeInterpolation, cvResizeOutput);

		// Step Blur0:
		Mat blurInput = cvResizeOutput;
		BlurType blurType = BlurType.get("Bilateral Filter");
		double blurRadius = 10.943396226415095;
		blur(blurInput, blurType, blurRadius, blurOutput);

		// Step RGB_Threshold0:
		Mat rgbThreshold0Input = blurOutput;
		double[] rgbThreshold0Red = {0.0, 43.63636363636365};
		double[] rgbThreshold0Green = {14.406779661016945, 255.0};
		double[] rgbThreshold0Blue = {0.0, 255.0};
		rgbThreshold(rgbThreshold0Input, rgbThreshold0Red, rgbThreshold0Green, rgbThreshold0Blue, rgbThreshold0Output);

		// Step HSV_Threshold0:
		Mat hsvThresholdInput = blurOutput;
		double[] hsvThresholdHue = {0.0, 101.39037433155079};
		double[] hsvThresholdSaturation = {216.10169491525423, 255.0};
		double[] hsvThresholdValue = {16.80790960451977, 255.0};
		hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

		// Step HSL_Threshold0:
		Mat hslThreshold0Input = blurOutput;
		double[] hslThreshold0Hue = {76.27118644067797, 94.97326203208556};
		double[] hslThreshold0Saturation = {220.90395480225988, 255.0};
		double[] hslThreshold0Luminance = {9.6045197740113, 82.27272727272727};
		hslThreshold(hslThreshold0Input, hslThreshold0Hue, hslThreshold0Saturation, hslThreshold0Luminance, hslThreshold0Output);

		// Step CV_bitwise_and0:
		Mat cvBitwiseAnd0Src1 = rgbThreshold0Output;
		Mat cvBitwiseAnd0Src2 = hsvThresholdOutput;
		cvBitwiseAnd(cvBitwiseAnd0Src1, cvBitwiseAnd0Src2, cvBitwiseAnd0Output);

		// Step CV_bitwise_and1:
		Mat cvBitwiseAnd1Src1 = hslThreshold0Output;
		Mat cvBitwiseAnd1Src2 = cvBitwiseAnd0Output;
		cvBitwiseAnd(cvBitwiseAnd1Src1, cvBitwiseAnd1Src2, cvBitwiseAnd1Output);

		// Step RGB_Threshold1:
		Mat rgbThreshold1Input = blurOutput;
		double[] rgbThreshold1Red = {0.0, 255.0};
		double[] rgbThreshold1Green = {0.0, 25.45454545454546};
		double[] rgbThreshold1Blue = {0.0, 235.0};
		rgbThreshold(rgbThreshold1Input, rgbThreshold1Red, rgbThreshold1Green, rgbThreshold1Blue, rgbThreshold1Output);

		// Step HSL_Threshold1:
		Mat hslThreshold1Input = blurOutput;
		double[] hslThreshold1Hue = {0.0, 180.0};
		double[] hslThreshold1Saturation = {0.0, 255.0};
		double[] hslThreshold1Luminance = {0.0, 11.81818181818182};
		hslThreshold(hslThreshold1Input, hslThreshold1Hue, hslThreshold1Saturation, hslThreshold1Luminance, hslThreshold1Output);

		// Step CV_bitwise_and2:
		Mat cvBitwiseAnd2Src1 = rgbThreshold1Output;
		Mat cvBitwiseAnd2Src2 = hslThreshold1Output;
		cvBitwiseAnd(cvBitwiseAnd2Src1, cvBitwiseAnd2Src2, cvBitwiseAnd2Output);

		// Step CV_subtract0:
		Mat cvSubtractSrc1 = cvBitwiseAnd1Output;
		Mat cvSubtractSrc2 = cvBitwiseAnd2Output;
		cvSubtract(cvSubtractSrc1, cvSubtractSrc2, cvSubtractOutput);

		// Step Mask0:
		Mat maskInput = cvResizeOutput;
		Mat maskMask = cvSubtractOutput;
		mask(maskInput, maskMask, maskOutput);

		// Step Find_Contours0:
		Mat findContoursInput = cvSubtractOutput;
		boolean findContoursExternalOnly = false;
		findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

		// Step Filter_Contours0:
		ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
		double filterContoursMinArea = 5.0;
		double filterContoursMinPerimeter = 5.0;
		double filterContoursMinWidth = 0.0;
		double filterContoursMaxWidth = 10000.0;
		double filterContoursMinHeight = 0.0;
		double filterContoursMaxHeight = 10000.0;
		double[] filterContoursSolidity = {57.43879472693031, 100.0};
		double filterContoursMaxVertices = 100.0;
		double filterContoursMinVertices = 2.0;
		double filterContoursMinRatio = 0.0;
		double filterContoursMaxRatio = 1000.0;
		filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);

		// Step Convex_Hulls0:
		ArrayList<MatOfPoint> convexHullsContours = filterContoursOutput;
		convexHulls(convexHullsContours, convexHullsOutput);

	}

	/**
	 * This method is a generated getter for the output of a CV_resize.
	 * @return Mat output from CV_resize.
	 */
	public Mat cvResizeOutput() {
		return cvResizeOutput;
	}

	/**
	 * This method is a generated getter for the output of a Blur.
	 * @return Mat output from Blur.
	 */
	public Mat blurOutput() {
		return blurOutput;
	}

	/**
	 * This method is a generated getter for the output of a RGB_Threshold.
	 * @return Mat output from RGB_Threshold.
	 */
	public Mat rgbThreshold0Output() {
		return rgbThreshold0Output;
	}

	/**
	 * This method is a generated getter for the output of a HSV_Threshold.
	 * @return Mat output from HSV_Threshold.
	 */
	public Mat hsvThresholdOutput() {
		return hsvThresholdOutput;
	}

	/**
	 * This method is a generated getter for the output of a HSL_Threshold.
	 * @return Mat output from HSL_Threshold.
	 */
	public Mat hslThreshold0Output() {
		return hslThreshold0Output;
	}

	/**
	 * This method is a generated getter for the output of a CV_bitwise_and.
	 * @return Mat output from CV_bitwise_and.
	 */
	public Mat cvBitwiseAnd0Output() {
		return cvBitwiseAnd0Output;
	}

	/**
	 * This method is a generated getter for the output of a CV_bitwise_and.
	 * @return Mat output from CV_bitwise_and.
	 */
	public Mat cvBitwiseAnd1Output() {
		return cvBitwiseAnd1Output;
	}

	/**
	 * This method is a generated getter for the output of a RGB_Threshold.
	 * @return Mat output from RGB_Threshold.
	 */
	public Mat rgbThreshold1Output() {
		return rgbThreshold1Output;
	}

	/**
	 * This method is a generated getter for the output of a HSL_Threshold.
	 * @return Mat output from HSL_Threshold.
	 */
	public Mat hslThreshold1Output() {
		return hslThreshold1Output;
	}

	/**
	 * This method is a generated getter for the output of a CV_bitwise_and.
	 * @return Mat output from CV_bitwise_and.
	 */
	public Mat cvBitwiseAnd2Output() {
		return cvBitwiseAnd2Output;
	}

	/**
	 * This method is a generated getter for the output of a CV_subtract.
	 * @return Mat output from CV_subtract.
	 */
	public Mat cvSubtractOutput() {
		return cvSubtractOutput;
	}

	/**
	 * This method is a generated getter for the output of a Mask.
	 * @return Mat output from Mask.
	 */
	public Mat maskOutput() {
		return maskOutput;
	}

	/**
	 * This method is a generated getter for the output of a Find_Contours.
	 * @return ArrayList<MatOfPoint> output from Find_Contours.
	 */
	public ArrayList<MatOfPoint> findContoursOutput() {
		return findContoursOutput;
	}

	/**
	 * This method is a generated getter for the output of a Filter_Contours.
	 * @return ArrayList<MatOfPoint> output from Filter_Contours.
	 */
	public ArrayList<MatOfPoint> filterContoursOutput() {
		return filterContoursOutput;
	}

	/**
	 * This method is a generated getter for the output of a Convex_Hulls.
	 * @return ArrayList<MatOfPoint> output from Convex_Hulls.
	 */
	public ArrayList<MatOfPoint> convexHullsOutput() {
		return convexHullsOutput;
	}


	/**
	 * Resizes an image.
	 * @param src The image to resize.
	 * @param dSize size to set the image.
	 * @param fx scale factor along X axis.
	 * @param fy scale factor along Y axis.
	 * @param interpolation type of interpolation to use.
	 * @param dst output image.
	 */
	private void cvResize(Mat src, Size dSize, double fx, double fy, int interpolation,
		Mat dst) {
		if (dSize==null) {
			dSize = new Size(0,0);
		}
		Imgproc.resize(src, dst, dSize, fx, fy, interpolation);
	}

	/**
	 * An indication of which type of filter to use for a blur.
	 * Choices are BOX, GAUSSIAN, MEDIAN, and BILATERAL
	 */
	enum BlurType{
		BOX("Box Blur"), GAUSSIAN("Gaussian Blur"), MEDIAN("Median Filter"),
			BILATERAL("Bilateral Filter");

		private final String label;

		BlurType(String label) {
			this.label = label;
		}

		public static BlurType get(String type) {
			if (BILATERAL.label.equals(type)) {
				return BILATERAL;
			}
			else if (GAUSSIAN.label.equals(type)) {
			return GAUSSIAN;
			}
			else if (MEDIAN.label.equals(type)) {
				return MEDIAN;
			}
			else {
				return BOX;
			}
		}

		@Override
		public String toString() {
			return this.label;
		}
	}

	/**
	 * Softens an image using one of several filters.
	 * @param input The image on which to perform the blur.
	 * @param type The blurType to perform.
	 * @param doubleRadius The radius for the blur.
	 * @param output The image in which to store the output.
	 */
	private void blur(Mat input, BlurType type, double doubleRadius,
		Mat output) {
		int radius = (int)(doubleRadius + 0.5);
		int kernelSize;
		switch(type){
			case BOX:
				kernelSize = 2 * radius + 1;
				Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
				break;
			case GAUSSIAN:
				kernelSize = 6 * radius + 1;
				Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);
				break;
			case MEDIAN:
				kernelSize = 2 * radius + 1;
				Imgproc.medianBlur(input, output, kernelSize);
				break;
			case BILATERAL:
				Imgproc.bilateralFilter(input, output, -1, radius, radius);
				break;
		}
	}

	/**
	 * Segment an image based on hue, saturation, and value ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue
	 * @param sat The min and max saturation
	 * @param val The min and max value
	 * @param output The image in which to store the output.
	 */
	private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
	    Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
		Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
			new Scalar(hue[1], sat[1], val[1]), out);
	}

	/**
	 * Segment an image based on color ranges.
	 * @param input The image on which to perform the RGB threshold.
	 * @param red The min and max red.
	 * @param green The min and max green.
	 * @param blue The min and max blue.
	 * @param output The image in which to store the output.
	 */
	private void rgbThreshold(Mat input, double[] red, double[] green, double[] blue,
		Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2RGB);
		Core.inRange(out, new Scalar(red[0], green[0], blue[0]),
			new Scalar(red[1], green[1], blue[1]), out);
	}

	/**
	 * Segment an image based on hue, saturation, and luminance ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue
	 * @param sat The min and max saturation
	 * @param lum The min and max luminance
	 * @param output The image in which to store the output.
	 */
	private void hslThreshold(Mat input, double[] hue, double[] sat, double[] lum,
		Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HLS);
		Core.inRange(out, new Scalar(hue[0], lum[0], sat[0]),
			new Scalar(hue[1], lum[1], sat[1]), out);
	}

	/**
	 * Computes the per channel and of two images.
	 * @param src1 The first image to use.
	 * @param src2 The second image to use.
	 * @param dst the result image when the and is performed.
	 */
	private void cvBitwiseAnd(Mat src1, Mat src2, Mat dst) {
		Core.bitwise_and(src1, src2, dst);
	}

	/**
	 * Subtracts the second Mat from the first.
	 * @param src1 the first Mat
	 * @param src2 the second Mat
	 * @param out the Mat that is the subtraction of the two Mats
	 */
	private void cvSubtract(Mat src1, Mat src2, Mat out) {
		Core.subtract(src1, src2, out);
	}

	/**
	 * Filter out an area of an image using a binary mask.
	 * @param input The image on which the mask filters.
	 * @param mask The binary image that is used to filter.
	 * @param output The image in which to store the output.
	 */
	private void mask(Mat input, Mat mask, Mat output) {
		mask.convertTo(mask, CvType.CV_8UC1);
		Core.bitwise_xor(output, output, output);
		input.copyTo(output, mask);
	}

	/**
	 * Sets the values of pixels in a binary image to their distance to the nearest black pixel.
	 * @param input The image on which to perform the Distance Transform.
	 * @param type The Transform.
	 * @param maskSize the size of the mask.
	 * @param output The image in which to store the output.
	 */
	private void findContours(Mat input, boolean externalOnly,
		List<MatOfPoint> contours) {
		Mat hierarchy = new Mat();
		contours.clear();
		int mode;
		if (externalOnly) {
			mode = Imgproc.RETR_EXTERNAL;
		}
		else {
			mode = Imgproc.RETR_LIST;
		}
		int method = Imgproc.CHAIN_APPROX_SIMPLE;
		Imgproc.findContours(input, contours, hierarchy, mode, method);
	}


	/**
	 * Filters out contours that do not meet certain criteria.
	 * @param inputContours is the input list of contours
	 * @param output is the the output list of contours
	 * @param minArea is the minimum area of a contour that will be kept
	 * @param minPerimeter is the minimum perimeter of a contour that will be kept
	 * @param minWidth minimum width of a contour
	 * @param maxWidth maximum width
	 * @param minHeight minimum height
	 * @param maxHeight maximimum height
	 * @param Solidity the minimum and maximum solidity of a contour
	 * @param minVertexCount minimum vertex Count of the contours
	 * @param maxVertexCount maximum vertex Count
	 * @param minRatio minimum ratio of width to height
	 * @param maxRatio maximum ratio of width to height
	 */
	private void filterContours(List<MatOfPoint> inputContours, double minArea,
		double minPerimeter, double minWidth, double maxWidth, double minHeight, double
		maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
		minRatio, double maxRatio, List<MatOfPoint> output) {
		final MatOfInt hull = new MatOfInt();
		output.clear();
		//operation
		for (int i = 0; i < inputContours.size(); i++) {
			final MatOfPoint contour = inputContours.get(i);
			final Rect bb = Imgproc.boundingRect(contour);
			if (bb.width < minWidth || bb.width > maxWidth) continue;
			if (bb.height < minHeight || bb.height > maxHeight) continue;
			final double area = Imgproc.contourArea(contour);
			if (area < minArea) continue;
			if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
			Imgproc.convexHull(contour, hull);
			MatOfPoint mopHull = new MatOfPoint();
			mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
			for (int j = 0; j < hull.size().height; j++) {
				int index = (int)hull.get(j, 0)[0];
				double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
				mopHull.put(j, 0, point);
			}
			final double solid = 100 * area / Imgproc.contourArea(mopHull);
			if (solid < solidity[0] || solid > solidity[1]) continue;
			if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
			final double ratio = bb.width / (double)bb.height;
			if (ratio < minRatio || ratio > maxRatio) continue;
			output.add(contour);
		}
	}

	/**
	 * Compute the convex hulls of contours.
	 * @param inputContours The contours on which to perform the operation.
	 * @param outputContours The contours where the output will be stored.
	 */
	private void convexHulls(List<MatOfPoint> inputContours,
		ArrayList<MatOfPoint> outputContours) {
		final MatOfInt hull = new MatOfInt();
		outputContours.clear();
		for (int i = 0; i < inputContours.size(); i++) {
			final MatOfPoint contour = inputContours.get(i);
			final MatOfPoint mopHull = new MatOfPoint();
			Imgproc.convexHull(contour, hull);
			mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
			for (int j = 0; j < hull.size().height; j++) {
				int index = (int) hull.get(j, 0)[0];
				double[] point = new double[] {contour.get(index, 0)[0], contour.get(index, 0)[1]};
				mopHull.put(j, 0, point);
			}
			outputContours.add(mopHull);
		}
	}




}

