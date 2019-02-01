//
// Inputs
//
Inputs
{
	Mat source0;
}

//
// Variables
//
Outputs
{
	Mat hsvThresholdOutput;
	ContoursReport findContoursOutput;
	ContoursReport filterContoursOutput;
}

//
// Steps
//

Step HSV_Threshold0
{
    Mat hsvThresholdInput = source0;
    List hsvThresholdHue = [53.41726618705036, 103.20819112627989];
    List hsvThresholdSaturation = [178.86690647482013, 255.0];
    List hsvThresholdValue = [77.96762589928058, 255.0];

    hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);
}

Step Find_Contours0
{
    Mat findContoursInput = hsvThresholdOutput;
    Boolean findContoursExternalOnly = true;

    findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);
}

Step Filter_Contours0
{
    ContoursReport filterContoursContours = findContoursOutput;
    Double filterContoursMinArea = 500.0;
    Double filterContoursMinPerimeter = 0;
    Double filterContoursMinWidth = 0;
    Double filterContoursMaxWidth = 1000;
    Double filterContoursMinHeight = 0;
    Double filterContoursMaxHeight = 1000;
    List filterContoursSolidity = [0.0, 100.0];
    Double filterContoursMaxVertices = 1000000;
    Double filterContoursMinVertices = 0;
    Double filterContoursMinRatio = 0.3;
    Double filterContoursMaxRatio = 0.9;

    filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);
}

Step NTPublish_ContoursReport0
{
    ContoursReport ntpublishContoursreportData = filterContoursOutput;
    String ntpublishContoursreportName = "contourReport";
    Boolean ntpublishContoursreportPublishArea = true;
    Boolean ntpublishContoursreportPublishCenterx = true;
    Boolean ntpublishContoursreportPublishCentery = true;
    Boolean ntpublishContoursreportPublishWidth = true;
    Boolean ntpublishContoursreportPublishHeight = true;
    Boolean ntpublishContoursreportPublishSolidity = true;

    ntpublishContoursreport(ntpublishContoursreportData, ntpublishContoursreportName, ntpublishContoursreportPublishArea, ntpublishContoursreportPublishCenterx, ntpublishContoursreportPublishCentery, ntpublishContoursreportPublishWidth, ntpublishContoursreportPublishHeight, ntpublishContoursreportPublishSolidity, );
}




