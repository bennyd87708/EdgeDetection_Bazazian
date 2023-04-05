// main.cpp : Defines the entry point for the application.
//

#include "main.h"
#include <iomanip>
#include <ctime>
#include <E57SimpleReader.h>
#include <E57SimpleWriter.h>

using namespace std;

double randDouble(int max) {
    return max * rand() / (RAND_MAX + 1.0f);
}

void readTestFile() {
    cout << "\n\nREADING TEST.E57\n";
    e57::Reader reader("../../../../data/test.e57", {});
    e57::E57Root header;
    reader.GetE57Root(header);
    cout << "GUID: " << header.guid << "\n";
    //reader.getdata3dcount()
    e57::Data3D data3DHeader;
    reader.ReadData3D(0, data3DHeader);
    cout << "Data GUID: " << data3DHeader.guid << "\n";
    cout << "Point Count: " << data3DHeader.pointCount << "\n";
    size_t totalPoints = data3DHeader.pointCount;
    e57::Data3DPointsDouble pointsData(data3DHeader);
    auto vectorReader = reader.SetUpData3DPointsData(0, totalPoints, pointsData);
    vectorReader.read();
    vectorReader.close();
    for (int i = 0; i < totalPoints; i++) {
        printf("Point #%-6dLocation: %-13f%-13f%-15fColor: %-5d%-5d%-5d\n", i,
            pointsData.cartesianX[i], pointsData.cartesianY[i], pointsData.cartesianZ[i],
            pointsData.colorRed[i], pointsData.colorGreen[i], pointsData.colorBlue[i]);
    }
    reader.Close();
}

void writeTestFile() {
    cout << "WRITING TEST.E57\n";
    e57::WriterOptions options;
    options.guid = "Test File GUID";
    e57::Writer writer("../../../../data/test.e57", options);
    e57::Data3D	header;
    size_t totalPoints = 5;
    int maxCoord = 1024;
    header.guid = "Test File Scan Header GUID";
    header.description = "libE57Format test write file";
    header.pointCount = totalPoints;
    header.pointFields.cartesianXField = true;
    header.pointFields.cartesianYField = true;
    header.pointFields.cartesianZField = true;
    header.pointFields.colorRedField = true;
    header.pointFields.colorGreenField = true;
    header.pointFields.colorBlueField = true;
    header.colorLimits.colorRedMaximum = 255;
    header.colorLimits.colorGreenMaximum = 255;
    header.colorLimits.colorBlueMaximum = 255;
    header.pointFields.pointRangeMinimum = 0.0;
    header.pointFields.pointRangeMaximum = double(maxCoord);
    e57::Data3DPointsDouble pointsData(header);
    for (int i = 0; i < totalPoints; i++) {
        pointsData.cartesianX[i] = randDouble(maxCoord);
        pointsData.cartesianY[i] = randDouble(maxCoord);
        pointsData.cartesianZ[i] = randDouble(maxCoord);
        pointsData.colorRed[i] = int(randDouble(255));
        pointsData.colorGreen[i] = int(randDouble(255));
        pointsData.colorBlue[i] = int(randDouble(255));

        printf("Point #%-6dLocation: %-13f%-13f%-15fColor: %-5d%-5d%-5d\n", i,
            pointsData.cartesianX[i], pointsData.cartesianY[i], pointsData.cartesianZ[i],
            pointsData.colorRed[i], pointsData.colorGreen[i], pointsData.colorBlue[i]);
    }
    try {
        writer.WriteData3DData(header, pointsData);
    }
    catch (e57::E57Exception& err)
    {
        cout << err.errorStr() << ": " << err.context();
    }
    writer.Close();
}

int main()
{
    srand(time(NULL));
    writeTestFile();
    readTestFile();
    cin.get();
    return 0;
}