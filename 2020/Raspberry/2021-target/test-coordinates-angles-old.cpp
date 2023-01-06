#include "GripPipeline.h"
#include<cstdio>

namespace grip {
    
    GripPipeline::GripPipeline() {
    }
    /**
     * Runs an iteration of the pipeline and updates outputs.
     */
    void GripPipeline::Process(cv::Mat& source0){
        //Step HSV_Threshold0:
        //input
        cv::Mat hsvThresholdInput = source0;
        double hsvThresholdHue[] = {0.0, 83.0824562042273};
        double hsvThresholdSaturation[] = {2.0564516428132875, 31.652964716548507};
        double hsvThresholdValue[] = {181.39681292769228, 255.0};
        hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, this->hsvThresholdOutput);
        //Step Find_Lines0:
        //input
        cv::Mat findLinesInput = hsvThresholdOutput;
        findLines(findLinesInput, this->findLinesOutput);
        //Step Filter_Lines0:
        //input
        std::vector<Line> filterLines0Lines = findLinesOutput;
        double filterLines0MinLength = 7.2;  // default Double
        double filterLines0Angle[] = {23.342032286144494, 127.86525641459997};
        filterLines(filterLines0Lines, filterLines0MinLength, filterLines0Angle, this->filterLines0Output);
        //Step Filter_Lines1:
        //input
        std::vector<Line> filterLines1Lines = filterLines0Output;
        double filterLines1MinLength = 8.9;  // default Double
        double filterLines1Angle[] = {68.75925100317322, 315.78850446147726};
        filterLines(filterLines1Lines, filterLines1MinLength, filterLines1Angle, this->filterLines1Output);
        //Step Filter_Lines2:
        //input
        std::vector<Line> filterLines2Lines = filterLines1Output;
        double filterLines2MinLength = 8.9;  // default Double
        double filterLines2Angle[] = {54.20566658234254, 114.52975746568906};
        filterLines(filterLines2Lines, filterLines2MinLength, filterLines2Angle, this->filterLines2Output);
	Printvalues(this->filterLines2Output);
    }
    
    /**
     * This method is a generated getter for the output of a HSV_Threshold.
     * @return Mat output from HSV_Threshold.
     */
    cv::Mat* GripPipeline::GetHsvThresholdOutput(){
        return &(this->hsvThresholdOutput);
    }
    /**
     * This method is a generated getter for the output of a Find_Lines.
     * @return LinesReport output from Find_Lines.
     */
    std::vector<Line>* GripPipeline::GetFindLinesOutput(){
        return &(this->findLinesOutput);
    }
    /**
     * This method is a generated getter for the output of a Filter_Lines.
     * @return LinesReport output from Filter_Lines.
     */
    std::vector<Line>* GripPipeline::GetFilterLines0Output(){
        return &(this->filterLines0Output);
    }
    /**
     * This method is a generated getter for the output of a Filter_Lines.
     * @return LinesReport output from Filter_Lines.
     */
    std::vector<Line>* GripPipeline::GetFilterLines1Output(){
        return &(this->filterLines1Output);
    }
    /**
     * This method is a generated getter for the output of a Filter_Lines.
     * @return LinesReport output from Filter_Lines.
     */
    std::vector<Line>* GripPipeline::GetFilterLines2Output(){
        return &(this->filterLines2Output);
    }
    /**
     * Segment an image based on hue, saturation, and value ranges.
     *
     * @param input The image on which to perform the HSL threshold.
     * @param hue The min and max hue.
     * @param sat The min and max saturation.
     * @param val The min and max value.
     * @param output The image in which to store the output.
     */
    void GripPipeline::hsvThreshold(cv::Mat &input, double hue[], double sat[], double val[], cv::Mat &out) {
        cv::cvtColor(input, out, cv::COLOR_BGR2HSV);
        cv::inRange(out,cv::Scalar(hue[0], sat[0], val[0]), cv::Scalar(hue[1], sat[1], val[1]), out);
    }
    
    /**
     * Finds all line segments in an image.
     *
     * @param input The image on which to perform the find lines.
     * @param lineList The output where the lines are stored.
     */
    void GripPipeline::findLines(cv::Mat &input, std::vector<Line> &lineList) {
        cv::Ptr<cv::LineSegmentDetector> lsd = cv::createLineSegmentDetector(cv::LSD_REFINE_STD);
        std::vector<cv::Vec4i> lines;
        lineList.clear();
        if (input.channels() == 1) {
            lsd->detect(input, lines);
        } else {
            // The line detector works on a single channel.
            cv::Mat tmp;
            cv::cvtColor(input, tmp, cv::COLOR_BGR2GRAY);
            lsd->detect(tmp, lines);
        }
        // Store the lines in the LinesReport object
        if (!lines.empty()) {
            for (int i = 0; i < lines.size(); i++) {
                cv::Vec4i line = lines[i];
                lineList.push_back(Line(line[0], line[1], line[2], line[3]));
            }
        }
    }
    
    /**
     * Filters out lines that do not meet certain criteria.
     *
     * @param inputs The lines that will be filtered.
     * @param minLength The minimum length of a line to be kept.
     * @param angle The minimum and maximum angle of a line to be kept.
     * @param outputs The output lines after the filter.
     */
    void GripPipeline::filterLines(std::vector<Line> &inputs, double minLength, double angle[], std::vector<Line> &outputs) {
        outputs.clear();
        for (Line line: inputs) {
            if (line.length()>abs(minLength)) {
                if ((line.angle() >= angle[0] && line.angle() <= angle[1]) ||
                    (line.angle() + 180.0 >= angle[0] && line.angle() + 180.0 <=angle[1])) {
                    outputs.push_back(line);
                }
            }
        }
    }
    
    
    void GripPipeline::Printvalues(std::vector<Line> &inputs) {
        
        for (int i = 0; i < inputs.size(); i++) {
            double angle1 = inputs[i].angle();
            double length1 = inputs[i].length();
            double x1, y1, x2, y2;
            x1 = inputs[i].x1;
            y1 = inputs[i].y1;
            x2 = inputs[i].x2;
            y2 = inputs[i].y2;
            printf("line%d:\n",i);
            printf("angle:%lf\n",angle1);
            printf("length:%lf\n",length1);
            printf("line[0](x1):%lf\n",x1);
            printf("line[1](y1):%lf\n",y1);
            printf("line[2](x2):%lf\n",x2);
            printf("line[3](y2):%lf\n\n",y2);
            
            }
    }
    
    
} // end grip namespace


