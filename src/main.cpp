// main.cpp : Defines the entry point for the application.
//

#include "main.h"
#include <eigen/dense>
#include <eigen/eigenvalues>
#include <iomanip>
#include <ctime>
#include <E57SimpleReader.h>
#include <E57SimpleWriter.h>

using namespace e57;
using namespace Eigen;
using namespace std;

static int POINTS = 5000;

double randDouble(int max) {
    return max * rand() / (RAND_MAX + 1.0f);
}

void setUsingColoredCartesianPoints(Data3D& header)
{
    header.pointFields.cartesianXField = true;
    header.pointFields.cartesianYField = true;
    header.pointFields.cartesianZField = true;

    header.pointFields.colorRedField = true;
    header.pointFields.colorGreenField = true;
    header.pointFields.colorBlueField = true;

    header.colorLimits.colorRedMaximum = 255;
    header.colorLimits.colorGreenMaximum = 255;
    header.colorLimits.colorBlueMaximum = 255;
}

void writeTestFile() {
    cout << "WRITING TEST.E57\n";
    srand(time(NULL));
    WriterOptions options;
    options.guid = "Test File GUID";
    Writer writer("../../../../data/test.e57", options);
    Data3D header;
    int maxCoord = 1024;
    setUsingColoredCartesianPoints(header);
    header.guid = "Test File Scan Header GUID";
    header.description = "libE57Format test write file";
    header.pointCount = POINTS;
    header.pointFields.pointRangeMinimum = 0.0;
    header.pointFields.pointRangeMaximum = double(maxCoord);
    Data3DPointsDouble pointsData(header);
    for (int i = 0; i < POINTS; i++) {
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
    catch (E57Exception& err)
    {
        cout << err.errorStr() << ": " << err.context();
    }
    writer.Close();
}

Data3DPointsDouble readFile(string filename) {
    cout << "\n\nREADING " << filename << "\n";
    string location = "../../../../data/" + filename;
    Reader reader(location, {});

    E57Root header;
    reader.GetE57Root(header);
    cout << "GUID: " << header.guid << "\n";
    int blockCount = reader.GetData3DCount();
    cout << "Block Count: " << blockCount << "\n";

    Data3D data3DHeader;
    for (int b = 0; b < blockCount; b++) {
        reader.ReadData3D(b, data3DHeader);
        int pointCount = data3DHeader.pointCount;
        cout << "Block GUID: " << data3DHeader.guid << "\n";
        cout << "Point Count: " << pointCount << "\n";

        Data3DPointsDouble pointsData(data3DHeader);
        CompressedVectorReader vectorReader = reader.SetUpData3DPointsData(0, pointCount, pointsData);

        vectorReader.read();
        vectorReader.close();
        for (int i = 0; i < pointCount; i++) {
            printf("Point #%-6dLocation: %-13f%-13f%-15fColor: %-5d%-5d%-5d\n", i,
                pointsData.cartesianX[i], pointsData.cartesianY[i], pointsData.cartesianZ[i],
                pointsData.colorRed[i], pointsData.colorGreen[i], pointsData.colorBlue[i]);
        }
        reader.Close();
        return pointsData;
    }
}

Data3DPointsDouble markEdges(Data3DPointsDouble data) {
    MatrixXd mat(POINTS, 3);
    for (int i = 0; i < POINTS; i++) {
        mat(i, 0) = data.cartesianX[i];
        mat(i, 1) = data.cartesianY[i];
        mat(i, 2) = data.cartesianZ[i];
    }
    Vector3d mean = mat.colwise().mean();

    VectorXd variations(POINTS);
    for (int i = 0; i < POINTS; i++) {
        double x = mat(i, 0) - mean[0];
        double y = mat(i, 1) - mean[1];
        double z = mat(i, 2) - mean[2];
        MatrixXd cov{
            {x * x, x * y, x * z},
            {y * x, y * y, y * z},
            {z * x, z * y, z * z}
        };

        EigenSolver<Matrix3d> es(cov);
        Matrix3d D = es.pseudoEigenvalueMatrix();
        double lambda_0 = D(0, 0);
        double lambda_1 = D(1, 1);
        double lambda_2 = D(2, 2);

        variations[i] = lambda_0 / (lambda_0 + lambda_1 + lambda_2);

        if (variations[i] > 0.99) {
            data.colorRed[i] = 255;
            data.colorBlue[i] = 255;
            data.colorGreen[i] = 255;
        }
        else {
            data.colorRed[i] = 1;
            data.colorBlue[i] = 1;
            data.colorGreen[i] = 1;
        }
    }
    return data;
}

void writeDataToFile(string filename, Data3DPointsDouble data) {
    WriterOptions options;
    options.guid = "Test File GUID";
    cout << "\n\nWRITING " << filename << "\n";
    string location = "../../../../data/" + filename;
    Writer writer(location, options);
    Data3D header;
    setUsingColoredCartesianPoints(header);
    header.guid = "Test File Scan Header GUID";
    header.description = "libE57Format test write file";
    header.pointCount = POINTS;
    header.pointFields.pointRangeMinimum = 0.0;
    header.pointFields.pointRangeMaximum = 1024.0;

    writer.WriteData3DData(header, data);
    writer.Close();
}

int main() {
    writeTestFile();
    Data3DPointsDouble data = readFile("test.e57");
    data = markEdges(data);
    writeDataToFile("test.e57", data);
    readFile("test.e57");

    cin.get();
    return 0;
}