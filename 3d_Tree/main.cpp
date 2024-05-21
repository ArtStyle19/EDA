#include <windows.h>
#include <windowsx.h>
#include <commctrl.h>
#include <stdio.h>
#include "resource.h"
#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <math.h>
#include <wingdi.h>
#include <gdiplus.h>

#pragma comment(lib, "Gdiplus.lib")




//GDI PLUS
void InitGDIPlus(ULONG_PTR *gdiplusToken)
{
    Gdiplus::GdiplusStartupInput gdiplusStartupInput;
    Gdiplus::GdiplusStartup(gdiplusToken, &gdiplusStartupInput, NULL);
}

void ShutdownGDIPlus(ULONG_PTR gdiplusToken)
{
    Gdiplus::GdiplusShutdown(gdiplusToken);
}

HINSTANCE hInst;
class Point
{
public:
    int x, y, z;
    Point(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z)
    {
    }
};

class Node
{
public:
    int depth;
    Point main_point;
    Node* children[2];
    Node(Point main, int depth = 0) : depth(depth)
    {
        main_point = main;
        children[0] = nullptr;
        children[1] = nullptr;
    }
};

class Comparison
{
public:
    int axis;
    Comparison (int axis = 0) : axis(axis) {};
    bool operator() (const Point &p1, const Point &p2)
    {
        if (axis == 0)
            return p1.x < p2.x;
        else if (axis == 1)
            return p1.y < p2.y;
        else
            return p1.z < p2.z;
    }
};




// Rotations

Point RotatePointX(Point& point, double angle)
{
    double s = sin(angle);
    double c = cos(angle);
    double newY = point.y * c - point.z * s;
    double newZ = point.y * s + point.z * c;
    return Point(point.x, newY, newZ);
}

Point RotatePointY(Point& point, double angle)
{
    double s = sin(angle);
    double c = cos(angle);
    double newX = point.x * c + point.z * s;
    double newZ = -point.x * s + point.z * c;
    return Point(newX, point.y, newZ);
}

Point RotatePointZ(Point& point, double angle)
{
    double s = sin(angle);
    double c = cos(angle);
    double newX = point.x * c - point.y * s;
    double newY = point.x * s + point.y * c;
    return Point(newX, newY, point.z);
}




double Combine(double a, double b, double factor) {
    return a * (1.0 - factor) + b * factor;
}


//Point RotatePointXYZ(Point& point, double angle1, double angle2, double angle3) {
//    // Rotar independient   emente alrededor de cada eje
//    Point rotatedX = RotatePointX(point, angle1);
//    Point rotatedY = RotatePointY(point, angle2);
//    Point rotatedZ = RotatePointZ(point, angle3);
//
//    // Combinar los resultados
//    Point combined;
//    combined.x = rotatedY.x; // Combina x de rotatedY y rotatedZ
//    combined.y = rotatedX.y; // Combina y de rotatedX y rotatedZ
//combined.z = rotatedY.z; // Combina z de rotatedX y rotatedY
//
//    return combined;
//}


Point RotatePointYAndZ(Point& point, double Elev, double Giro)
{
    // Calculate new x
    double x2D = point.x * cos(Elev) - point.z * sin(Elev);

    // Calculate new y based on both Elev and Giro
    double y2D = -point.x * sin(Elev) * sin(Giro) + point.y * cos(Giro) - point.z * cos(Elev) * sin(Giro);

    // For this combined rotation, z remains the same as the original point
    return Point(x2D, y2D, point.z);
}



//
Point RotatePoint(Point& point, double angleX, double angleY, double angleZ)
{

//Point rotated = RotatePointXYZ(point, angleX, angleY, angleZ);
//    rotated = RotatePointY(rotated, angleY);
//    rotated = RotatePointZ(rotated, angleZ);
//angleY = CalculateNewAngleY(rotated);
//angleZ = CalculateNewAngleZ(rotated);
//angleX = CalculateNewAngleX(rotated);

//    angleY = CalculateNewAngleY(rotated);
//angleZ = CalculateNewAngleZ(rotated);
//angleX = CalculateNewAngleX(rotated);

    Point rotated = RotatePointYAndZ(point, angleY, angleX);
//
//    rotated = RotatePointX(rotated, angleX);

    return rotated;
}



//
//
//
//
//class Matrix3x3
//{
//public:
//    double m[3][3];
//
//    Matrix3x3(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
//    {
//        m[0][0] = m00; m[0][1] = m01; m[0][2] = m02;
//        m[1][0] = m10; m[1][1] = m11; m[1][2] = m12;
//        m[2][0] = m20; m[2][1] = m21; m[2][2] = m22;
//    }
//
//    Point operator*(const Point& point) const
//    {
//        double newX = m[0][0] * point.x + m[0][1] * point.y + m[0][2] * point.z;
//        double newY = m[1][0] * point.x + m[1][1] * point.y + m[1][2] * point.z;
//        double newZ = m[2][0] * point.x + m[2][1] * point.y + m[2][2] * point.z;
//        return Point(newX, newY, newZ);
//    }
//
//    Matrix3x3 operator*(const Matrix3x3& other) const
//    {
//        Matrix3x3 result(0, 0, 0, 0, 0, 0, 0, 0, 0);
//        for (int i = 0; i < 3; ++i)
//        {
//            for (int j = 0; j < 3; ++j)
//            {
//                result.m[i][j] = m[i][0] * other.m[0][j] + m[i][1] * other.m[1][j] + m[i][2] * other.m[2][j];
//            }
//        }
//        return result;
//    }
//};
//Matrix3x3 RotationMatrixX(double angle)
//
//{
//    double s = sin(angle);
//    double c = cos(angle);
//    return Matrix3x3(
//        1, 0, 0,
//        0, c, -s,
//        0, s, c
//    );
//}
//
//Matrix3x3 RotationMatrixY(double angle)
//{
//    double s = sin(angle);
//    double c = cos(angle);
//    return Matrix3x3(
//        c, 0, s,
//        0, 1, 0,
//        -s, 0, c
//    );
//}
//
//Matrix3x3 RotationMatrixZ(double angle)
//{
//    double s = sin(angle);
//    double c = cos(angle);
//    return Matrix3x3(
//        c, -s, 0,
//        s, c, 0,
//        0, 0, 1
//    );
//}
//
//Point RotatePoint(const Point& point, double angleX, double angleY, double angleZ)
//{
//    Matrix3x3 rotationX = RotationMatrixX(angleX);
//    Matrix3x3 rotationY = RotationMatrixY(angleY);
//    Matrix3x3 rotationZ = RotationMatrixZ(angleZ);
//
//    // Combine the rotations
//    Matrix3x3 combinedRotation = rotationZ * rotationY * rotationX;
//
//    // Apply the combined rotation to the point
//    return combinedRotation * point;
//}
//


















// Tree


class KdTree
{
public:
    int k = 3;
    Node* root;
    KdTree()
    {
        root = nullptr;
    }

    void build(std::vector<Point> &points)
    {
        build_helper(points, root, 0);
    }

    void build_helper(std::vector<Point> &points, Node* &curr_node, int depth)
    {
        if (points.empty())
        {
            return;
        }

        int axis = depth % k; // Modulo 3 for 3D
        std::sort(points.begin(), points.end(), Comparison(axis));

        int median_index = points.size() / 2;
        curr_node = new Node(points[median_index], depth);

        std::vector<Point> left(points.begin(), points.begin() + median_index);
        std::vector<Point> right(points.begin() + median_index + 1, points.end());

        build_helper(left, curr_node->children[0], depth + 1);
        build_helper(right, curr_node->children[1], depth + 1);
    }
    void collectPoints(Node* node, std::vector<Point>& points)
    {
        if (node == nullptr) return;
        points.push_back(node->main_point);
        collectPoints(node->children[0], points);
        collectPoints(node->children[1], points);
    }

    void insertion(Point point)
    {
        insertion_helper(point, root, 0);
    }
    void insertion_helper(Point point, Node* &curr_node, int depth)
    {
        int axis = depth % k;
        if (curr_node == nullptr)
        {
            curr_node = new Node(point, depth);
            std::cout << curr_node->main_point.x << ' ' << curr_node->main_point.y << ' ' << curr_node->depth << '\n';
        }
        else
        {
            if (axis == 0)
            {
                if (point.x < curr_node->main_point.x)
                {
                    insertion_helper(point, curr_node->children[0], depth + 1);
                }
                else
                {
                    insertion_helper(point, curr_node->children[1], depth + 1);
                }
            }
            else
            {
                if (point.y < curr_node->main_point.y)
                {
                    insertion_helper(point, curr_node->children[0], depth + 1);
                }
                else
                {
                    insertion_helper(point, curr_node->children[1], depth + 1);
                }
            }
        }

        balance(curr_node, depth);
    }


    void balance(Node*& curr_node, int depth)
    {
        if (curr_node == nullptr) return;

        std::vector<Point> points;
        collectPoints(curr_node, points);
        int axis = depth % k;
        std::sort(points.begin(), points.end(), Comparison(axis));
        int median_index = points.size() / 2;

        curr_node->main_point = points[median_index];
        curr_node->depth = depth;

        if (points.size() > 1)
        {
            Node* left_child = nullptr;
            Node* right_child = nullptr;

            if (median_index > 0)
            {
                std::vector<Point> left(points.begin(), points.begin() + median_index);
                balanceHelper(left_child, left, depth + 1);
            }

            if (median_index + 1 < points.size())
            {
                std::vector<Point> right(points.begin() + median_index + 1, points.end());
                balanceHelper(right_child, right, depth + 1);
            }

            curr_node->children[0] = left_child;
            curr_node->children[1] = right_child;
        }
    }

    void balanceHelper(Node*& curr_node, std::vector<Point>& points, int depth)
    {
        if (points.empty())
        {
            curr_node = nullptr;
            return;
        }
        int axis = depth % k;
        int median_index = points.size() / 2;

        std::sort(points.begin(), points.end(), Comparison(axis));

        curr_node = new Node(points[median_index], depth);

        std::vector<Point> left(points.begin(), points.begin() + median_index);
        balanceHelper(curr_node->children[0], left, depth + 1);

        std::vector<Point> right(points.begin() + median_index + 1, points.end());
        balanceHelper(curr_node->children[1], right, depth + 1);
    }



    //////////// dx

    struct PointDistance
    {
        Point point;
        float distance;

        PointDistance(Point p, float d) : point(p), distance(d) {}

        // Overload < operator for priority queue
        bool operator<(const PointDistance& other) const
        {
            return distance < other.distance; // Min-heap based on distance
        }
    };

    std::vector<Point> nearest_points(Point pivot, int N)
    {
        std::priority_queue<PointDistance> pq; // Min-heap of PointDistance
        nearest_points_helper(pivot, root, 0, pq, N);

        std::vector<Point> result;
        while (!pq.empty())
        {
            result.push_back(pq.top().point);
            pq.pop();
        }
        return result;
    }

    void nearest_points_helper(Point pivot, Node* curr_node, int depth, std::priority_queue<PointDistance>& pq, int N)
    {
        if (curr_node == nullptr)
        {
            return;
        }
        int axis = depth % k;
        PointDistance curr_point_distance(curr_node->main_point, dis(pivot, curr_node->main_point));
        if (pq.size() < N)
        {
            pq.push(curr_point_distance);
        }
        else if (curr_point_distance.distance < pq.top().distance)
        {
            pq.pop();
            pq.push(curr_point_distance);
        }
        Node* good_side = nullptr;
        Node* bad_side = nullptr;
        if (axis == 0)
        {
            if (pivot.x < curr_node->main_point.x)
            {
                good_side = curr_node->children[0];
                bad_side = curr_node->children[1];
            }
            else
            {
                good_side = curr_node->children[1];
                bad_side = curr_node->children[0];
            }
        }
        else
        {
            if (pivot.y < curr_node->main_point.y)
            {
                good_side = curr_node->children[0];
                bad_side = curr_node->children[1];
            }
            else
            {
                good_side = curr_node->children[1];
                bad_side = curr_node->children[0];
            }
        }
        nearest_points_helper(pivot, good_side, depth + 1, pq, N);
        if (is_backtrack_needed(curr_node, pivot, pq.top().point, depth))
        {
            nearest_points_helper(pivot, bad_side, depth + 1, pq, N);
        }
    }
///////

    Point nearest_point(Point pivot)
    {
        return nearest_point_helper(pivot, root, 0);
    }
    float dis(const Point &p1, const Point &p2)
    {
        return sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y)) ;
    }

    Point nearest_point_helper(Point pivot, Node* curr_node, int depth)
    {
        if (curr_node == nullptr)
        {
            return Point(1000, 1000);
        }

        int axis = depth % k;

        Point best = curr_node->main_point;
        Node* good_side = nullptr;
        Node* bad_side = nullptr;

        if (axis == 0)
        {
            if (pivot.x < curr_node->main_point.x)
            {
                good_side = curr_node->children[0];
                bad_side = curr_node->children[1];
            }
            else
            {
                good_side = curr_node->children[1];
                bad_side = curr_node->children[0];
            }
        }
        else
        {
            if (pivot.y < curr_node->main_point.y)
            {
                good_side = curr_node->children[0];
                bad_side = curr_node->children[1];
            }
            else
            {
                good_side = curr_node->children[1];
                bad_side = curr_node->children[0];
            }
        }

        Point best_good_side = nearest_point_helper(pivot, good_side, depth + 1);

        if (best.x  == INT_MAX || best_good_side.x == INT_MAX)
        {
            return curr_node->main_point;
        }
        if (dis(pivot, best) > dis(pivot, best_good_side))
        {
            best = best_good_side;
        }
        if (is_backtrack_needed(curr_node, pivot, best, depth))
        {
            best = closest(pivot, best, nearest_point_helper(pivot, bad_side, depth + 1));
        }
        return best;
    }

    Point closest(Point pivot, Point p1, Point p2)
    {
        if (dis(pivot, p1) < dis(pivot, p2))
        {
            return p1;
        }
        else
        {
            return p2;
        }
    }

    bool is_backtrack_needed(Node* curr_node, Point pivot, Point best, int depth)
    {
        if (curr_node == nullptr) return false;
        int axis = depth % k;
        if (axis == 0)
        {
            if (curr_node->main_point.x > pivot.x && curr_node->main_point.x - pivot.x < dis(pivot, best))
                return true;

            if (curr_node->main_point.x < pivot.x && pivot.x - curr_node->main_point.x < dis(pivot, best))
                return true;
        }
        else
        {

            if (curr_node->main_point.y > pivot.y && curr_node->main_point.y - pivot.y < dis(pivot, best))
                return true;

            if (curr_node->main_point.y < pivot.y && pivot.y - curr_node->main_point.y < dis(pivot, best))
                return true;

        }
        return false;
    }

    void MostrarR(HDC hdc,int x,int y,int a)
    {
        MosR(root,hdc,x,y,a);
    }
    void MosR(Node *R,HDC hdc,int x,int y,int a)
    {
        if(R!=NULL)
        {
            char Cad[10];
            sprintf(Cad,"%i",R->main_point.x);
            TextOut(hdc,x,y,Cad,strlen(Cad));
            sprintf(Cad,"%i",R->main_point.y);
            TextOut(hdc,x,y+15,Cad,strlen(Cad));
            sprintf(Cad,"%i",R->main_point.z);
            TextOut(hdc,x,y+30,Cad,strlen(Cad));
            sprintf(Cad,"%i",R->depth % k);
            TextOut(hdc,x,y+45,Cad,strlen(Cad));
            MosR(R->children[0],hdc,x-a,y+50,a/2);
            MosR(R->children[1],hdc,x+a,y+50,a/2);
        }
    }


    void DrawLine(HDC hdc, int centerX, int centerY, const Point& p1, const Point& p2) {
    Point proj1 = Project(p1);
    Point proj2 = Project(p2);
    MoveToEx(hdc, centerX + proj1.x, centerY + proj1.y, NULL);
    LineTo(hdc, centerX + proj2.x, centerY + proj2.y);
}


//





// GDI opacidad


void DrawPolygonWithTransparency(HDC hdc, Point* polygonPoints, int numPoints, int depth, BYTE alpha)
{
    Gdiplus::GdiplusStartupInput gdiplusStartupInput;
    ULONG_PTR gdiplusToken;
    GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);
    Gdiplus::Graphics graphics(hdc);
    graphics.SetSmoothingMode(Gdiplus::SmoothingModeAntiAlias);
    int red = 120 * (depth % 3);
    int green = 120 * ((depth + 1) % 3);
    int blue = 120 * ((depth + 2) % 3);
    Gdiplus::Color color(alpha, red, green, blue);
    Gdiplus::SolidBrush brush(color);
    Gdiplus::PointF* gdiPoints = new Gdiplus::PointF[numPoints];
    for (int i = 0; i < numPoints; ++i) {
        gdiPoints[i] = Gdiplus::PointF(static_cast<float>(polygonPoints[i].x), static_cast<float>(polygonPoints[i].y));
    }
    graphics.FillPolygon(&brush, gdiPoints, numPoints);
    delete[] gdiPoints;
    Gdiplus::GdiplusShutdown(gdiplusToken);
}

void SetBrushColor(HDC hdc, int depth, Point* polygonPoints, int numPoints, BYTE alpha) {
    DrawPolygonWithTransparency(hdc, polygonPoints, numPoints, depth, alpha);
}




//
// Alpha blend


// Opacidad con alphablender libreria = Msimg32
// Opacidad
//void SetBrushColor2(HDC hdc, int depth, POINT* polygonPoints, int numPoints, BYTE alpha) {
//    // Create a memory DC for off-screen drawing
//HDC memDC = CreateCompatibleDC(hdc);
//HBITMAP memBitmap = CreateCompatibleBitmap(hdc, 400, 400); // Adjust width and height as needed
//HBITMAP oldBitmap = (HBITMAP)SelectObject(memDC, memBitmap);
//
//// Fill the entire destination bitmap with the desired background color (e.g., white)
//HBRUSH hBackgroundBrush = CreateSolidBrush(RGB(255, 255, 255)); // White color
//RECT drawingArea = { 0, 0, 400, 400}; // Define the drawing area
//FillRect(memDC, &drawingArea, hBackgroundBrush); // Fill the drawing area with white color
//DeleteObject(hBackgroundBrush); // Clean up the brush
//    // Create a solid brush with the specified color
//    int red = 120 * (depth % 3);
//    int green = 120 * ((depth + 1) % 3);
//    int blue = 120 * ((depth + 2) % 3);
//    HBRUSH hBrush = CreateSolidBrush(RGB(red, green, blue));
//
//    // Fill the polygon on the memory DC
//    SelectObject(memDC, hBrush);
//    Polygon(memDC, polygonPoints, numPoints);
//
//    // Set up the blend function for transparency
//    BLENDFUNCTION blendFunction;
//    blendFunction.BlendOp = AC_SRC_OVER;
//    blendFunction.BlendFlags = 0;
//    blendFunction.SourceConstantAlpha = 32;
//    blendFunction.AlphaFormat = 0;
//
//    // Perform the alpha blend
//    AlphaBlend(hdc, 0, 0, 400, 400, memDC, 0, 0, 400, 400, blendFunction); // Adjust size and position as needed
//
//    // Clean up
//    SelectObject(memDC, oldBitmap);
//    DeleteObject(memBitmap);
//    DeleteDC(memDC);
//    DeleteObject(hBrush);
//}



    void Mostrar(HDC hdc, int centerX, int centerY, double angleX, double angleY, double angleZ)
    {
        double initialx = 100; // Adjust as needed for initial range
        double initialy = 100; // Adjust as needed for initial range
        double initialz = 100; // Adjust as needed for initial range2

        Mos(root, hdc, centerX, centerY, angleX, angleY, angleZ, initialx, initialy, initialz, 0, 0, 0);
    }
    void Mos(Node* R, HDC hdc, int centerX, int centerY, double angleX, double angleY, double angleZ, double rangex, double rangey, double rangez, double a, double b,double c)
    {
        if (R != NULL)
        {
            Point rotatedPoint = R->main_point;

            // Apply rotations around X, Y, and Z axes
//            rotatedPoint = RotatePointX(rotatedPoint, angleX / 16);
//            rotatedPoint = RotatePointY(rotatedPoint, angleY / 16);
//            rotatedPoint = RotatePointZ(rotatedPoint, angleZ / 16);

            rotatedPoint = RotatePoint(rotatedPoint, angleX/16, angleY/16, angleZ/16);



            // Project the rotated point onto the 2D plane
            Point projected = Project(rotatedPoint);
            int xx = centerX + projected.x;
            int yy = centerY + projected.y;

            // Draw the point


//            int red = rand() % 256;
//            int green = rand() % 256;
//            int blue = rand() % 256;

            int red = 60 * (R->depth % 3);
            int green = 60 * ((R->depth + 1) % 3);
            int blue = 60 * ((R->depth + 2) % 3);
            HBRUSH hBrush = CreateSolidBrush(RGB(red, green, blue));
            SelectObject(hdc, hBrush);

            const int surfaceSize = 10;
            Ellipse(hdc, xx - surfaceSize / 2, yy - surfaceSize / 2, xx + surfaceSize / 2, yy + surfaceSize / 2);


            DeleteObject(hBrush);


            double rangexp;
            double rangeyp;
            double rangezp;

            double mini ;
            double maxa ;
            // Draw the splitting plane
            Point p1, p2, p3, p4;
            if (R->depth % 3 == 0)
            {

                // Split on x-axis, draw a vertical plane
                p1 = { R->main_point.x, b - abs(rangey), c - abs(rangez)};
                p2 = { R->main_point.x, b - abs(rangey), c + abs(rangez)};
                p3 = { R->main_point.x, b + abs(rangey), c + abs(rangez)}; // no siempre es 0 al principio si
                p4 = { R->main_point.x, b + abs(rangey), c - abs(rangez)};

                if (R->main_point.x >= 0) {
                    mini = std::min(abs(100 - R->main_point.x), abs(100 + R->main_point.x));
                    maxa = std::max(abs(100 - R->main_point.x), abs(100 + R->main_point.x));
                } else {
                    maxa = std::min(abs(100 - R->main_point.x), abs(100 + R->main_point.x));
                    mini = std::max(abs(100 - R->main_point.x), abs(100 + R->main_point.x));
                }


                rangexp = mini / 2; // Adjust the range for x-axis splitting
                rangey = rangey; // Adjust the range for x-axis splitting
                rangez = rangez; // Adjust the range for x-axis splitting

                rangex = maxa / 2;
                rangeyp = rangey;
                rangezp = rangez;


                a = R->main_point.x;

                if (R->main_point.x >= 0) {
                    Mos(R->children[0], hdc, centerX, centerY, angleX, angleY, angleZ, rangex, rangey, rangez, a, b, c); // eventualmente 0 y cambia
            Mos(R->children[1], hdc, centerX, centerY, angleX, angleY, angleZ, -rangexp, rangeyp, rangezp, a, b, c); // "
                } else {
                    Mos(R->children[1], hdc, centerX, centerY, angleX, angleY, angleZ, +rangex, rangey, rangez, a, b, c); // eventualmente 0 y cambia
            Mos(R->children[0], hdc, centerX, centerY, angleX, angleY, angleZ, -rangexp, rangeyp, rangezp, a, b, c); // "
                }

            }
            else if (R->depth % 3 == 1)
            {
                // Split on y-axis, draw a horizontal plane
                p1 = { a - rangex +abs(rangex), R->main_point.y, c - abs(rangez) };//fix
                p2 = { a - rangex +abs(rangex), R->main_point.y, c + abs(rangez)};
                p3 = { a - rangex -abs(rangex), R->main_point.y, c + abs(rangez) };
                p4 = { a - rangex - abs(rangex), R->main_point.y, c - abs(rangez) };

                if (R->main_point.y >= 0) {
                    mini = std::min(abs(100 - R->main_point.y), abs(100 + R->main_point.y));
                    maxa = std::max(abs(100 - R->main_point.y), abs(100 + R->main_point.y));
                } else {
                    maxa = std::min(abs(100 - R->main_point.y), abs(100 + R->main_point.y));
                    mini = std::max(abs(100 - R->main_point.y), abs(100 + R->main_point.y));
                }
                rangex = rangex; // Adjust the range for x-axis splitting
                rangeyp = mini /2; // Adjust the range for x-axis splitting
                rangez = rangez; // Adjust the range for x-axis splitting

                rangexp = rangex;
                rangey = maxa /2;
                rangezp = rangez;


                b = R->main_point.y;

                 if (R->main_point.y >= 0) {
                    Mos(R->children[0], hdc, centerX, centerY, angleX, angleY, angleZ, rangex, rangey, rangez, a, b, c); // eventualmente 0 y cambia
            Mos(R->children[1], hdc, centerX, centerY, angleX, angleY, angleZ, rangexp, -rangeyp, rangezp, a, b, c); // " cambiado
                } else {
                    Mos(R->children[0], hdc, centerX, centerY, angleX, angleY, angleZ, rangex, rangey, rangez, a, b, c); // eventualmente 0 y cambia
            Mos(R->children[1], hdc, centerX, centerY, angleX, angleY, angleZ, rangexp, -rangeyp, rangezp, a, b, c); // "
                }

            }

            else if (R->depth % 3 == 2)
            {
                // Split on z-axis, draw a depth plane
                p1 = { -rangex + a - abs(rangex), -rangey + b - abs(rangey), R->main_point.z };
                p2 = { -rangex + a - abs(rangex), -rangey + b + abs(rangey), R->main_point.z };
                p3 = { -rangex + a + abs(rangex), -rangey + b + abs(rangey), R->main_point.z };
                p4 = { -rangex + a +  abs(rangex), -rangey + b - abs(rangey), R->main_point.z };

                if (R->main_point.z >= 0) {
                    mini = std::min(abs(100 - R->main_point.z), abs(100 + R->main_point.z));
                    maxa = std::max(abs(100 - R->main_point.z), abs(100 + R->main_point.z));
                } else {
                    maxa = std::min(abs(100 - R->main_point.z), abs(100 + R->main_point.z));
                    mini = std::max(abs(100 - R->main_point.z), abs(100 + R->main_point.z));
                }
                rangex = rangex; // Adjust the range for x-axis splitting
                rangey = rangey; // Adjust the range for x-axis splitting
                rangezp = mini / 2; // Adjust the range for x-axis splitting //no es /2

                rangexp = rangex;
                rangeyp = rangey;
                rangez = maxa /2;

                c = R->main_point.z;

                 if (R->main_point.z >= 0) {
                    Mos(R->children[1], hdc, centerX, centerY, angleX, angleY, angleZ, rangex, rangey, rangez, a, b, c); // eventualmente 0 y cambia
            Mos(R->children[0], hdc, centerX, centerY, angleX, angleY, angleZ, rangexp, rangeyp, -rangezp, a, b, c); // "
                } else {
                    Mos(R->children[0], hdc, centerX, centerY, angleX, angleY, angleZ, rangex, rangey, rangez, a, b, c); // eventualmente 0 y cambia
            Mos(R->children[1], hdc, centerX, centerY, angleX, angleY, angleZ, rangexp, rangeyp, -rangezp, a, b, c); // "
                }
            }

            // Apply rotations to plane corners
//            p1 = RotatePointX(p1, angleX / 16);
//            p1 = RotatePointY(p1, angleY / 16);
//            p1 = RotatePointZ(p1, angleZ / 16);
//
//            p2 = RotatePointX(p2, angleX / 16);
//            p2 = RotatePointY(p2, angleY / 16);
//            p2 = RotatePointZ(p2, angleZ / 16);
//
//            p3 = RotatePointX(p3, angleX / 16);
//            p3 = RotatePointY(p3, angleY / 16);
//            p3 = RotatePointZ(p3, angleZ / 16);
//
//            p4 = RotatePointX(p4, angleX / 16);
//            p4 = RotatePointY(p4, angleY / 16);
//            p4 = RotatePointZ(p4, angleZ / 16);

            p1 = RotatePoint(p1, angleX/16, angleY/16, angleZ/16);
            p2 = RotatePoint(p2, angleX/16, angleY/16, angleZ/16);
            p3 = RotatePoint(p3, angleX/16, angleY/16, angleZ/16);
            p4 = RotatePoint(p4, angleX/16, angleY/16, angleZ/16);


            // Usando lineas
//            DrawLine(hdc, centerX, centerY, p1, p2);
//            DrawLine(hdc, centerX, centerY, p2, p3);
//            DrawLine(hdc, centerX, centerY, p3, p4);
//            DrawLine(hdc, centerX, centerY, p4, p1);




        Point polygonPoints[] = {
            { centerX + Project(p1).x, centerY + Project(p1).y },
            { centerX + Project(p2).x, centerY + Project(p2).y },
            { centerX + Project(p3).x, centerY + Project(p3).y },
            { centerX + Project(p4).x, centerY + Project(p4).y }
        };

        SetBrushColor(hdc, R->depth, polygonPoints, 4, 80); // 80% opacity


            DeleteObject(hBrush);

//        red = 100 * (R->depth % 3);
//    green = 100 * ((R->depth + 1) % 3);
//    blue = 100 * ((R->depth + 2) % 3);
//    hBrush = CreateSolidBrush(RGB(red, green, blue));
//    SelectObject(hdc, hBrush);
//
//    Polygon(hdc, polygonPoints, 4);



//            Mos(R->children[0], hdc, centerX, centerY, angleX, angleY, angleZ, rangex, rangey, rangez, a, b, c); // eventualmente 0 y cambia
//            Mos(R->children[1], hdc, centerX, centerY, angleX, angleY, angleZ, -rangexp, -rangeyp, -rangezp, a, b, c); // "
        }

    }

// Project function for orthogonal projection
    Point Project(Point p)
    {
        Point projected;
        projected.x = static_cast<int>(p.x);
        projected.y = static_cast<int>(p.y);
        return projected;
    }
};

//void DrawNumber(HWND hwnd, HDC hdc)
//{
//    char cad[100];
//    strcpy(cad, "100"); // Almacenar "100" en el array de caracteres
//    SetTextColor(hdc, RGB(0, 0, 0));
//    // Configurar el modo de dibujo
//    SetBkMode(hdc, TRANSPARENT); // Texto sin fondo
//    // Dibujar el número "100" en el centro de la ventana
//    TextOutA(hdc, 100, 100, cad, strlen(cad)); // Cambia las coordenadas según sea necesario
//}

void MostrarCercano(HDC hdc, Point n, Point target)
{
    HBRUSH hEllipseBrush = CreateSolidBrush(RGB(255, 0, 0));
    HBRUSH hEllipseBrush2 = CreateSolidBrush(RGB(0, 255, 0));

    HPEN hEllipsePen = CreatePen(PS_SOLID, 1, RGB(0,0,0));
    HPEN hEllipsePen2 = CreatePen(PS_SOLID, 1, RGB(0,0,0));


    SelectObject(hdc, hEllipseBrush);
    SelectObject(hdc, hEllipsePen);

    Ellipse(hdc, target.x - 4, target.y - 4, target.x + 4, target.y + 4);

    SelectObject(hdc, hEllipseBrush2);

    SelectObject(hdc, hEllipsePen2);

    std::cout << "(" << n.x << ", " << n.y << ")\n";
    Ellipse(hdc, n.x - 4, n.y - 4, n.x + 4, n.y + 4);

    DeleteObject(hEllipsePen);
    DeleteObject(hEllipsePen2);
    DeleteObject(hEllipseBrush);
}

void MostrarCercanos(HDC hdc, std::vector<Point> n, Point target)
{
    HBRUSH hEllipseBrush = CreateSolidBrush(RGB(255, 0, 0));
    HBRUSH hEllipseBrush2 = CreateSolidBrush(RGB(0, 0, 255));


    HPEN hEllipsePen = CreatePen(PS_SOLID, 1, RGB(0,0,0));
    HPEN hEllipsePen2 = CreatePen(PS_SOLID, 1, RGB(0,0,0));

    SelectObject(hdc, hEllipseBrush);
    SelectObject(hdc, hEllipsePen);

    Ellipse(hdc, target.x - 4, target.y - 4, target.x + 4, target.y + 4);

    SelectObject(hdc, hEllipseBrush2);

    SelectObject(hdc, hEllipsePen2);

    for (const auto& point : n)
    {
        Ellipse(hdc, point.x - 4, point.y - 4, point.x + 4, point.y + 4);

        // Calcula el radio basado en la distancia entre el punto y el objetivo
//        int diffx = abs(point.first - target.first);
//        int diffy = abs(point.second - target.second);
//        int radius = static_cast<int>(std::sqrt(diffx * diffx + diffy * diffy)); // Distancia euclidiana

        // Dibuja la circunferencia de la elipse centrada en el punto objetivo con el mismo radio
//        Arc(hdc, target.first - radius - 2, target.second - radius - 2, target.first + radius + 2, target.second + radius + 2, 0, 0, 0, 0);
    }
    DeleteObject(hEllipsePen);
    DeleteObject(hEllipsePen2);
    DeleteObject(hEllipseBrush);
}

void DrawCube(HDC hdc, int centerX, int centerY, int size, double angleX, double angleY, double angleZ)
{
    // Vertices of the cube with depth
    Point vertices[8] =
    {
        {-size, size, -size},
        {size, size, -size},
        {size, -size, -size},
        {-size, -size, -size},
        {-size, size, size},
        {size, size, size},
        {size, -size, size},
        {-size, -size, size}
    };

    // Rotate vertices
    for (int i = 0; i < 8; ++i)
    {
        vertices[i] = RotatePoint(vertices[i], angleX/16, angleY/16, angleZ/16);
    }

    // Define the lines forming the cube
    int lines[12][2] =
    {
        {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
        {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}
    };

    // Draw lines of the cube
    for (int i = 0; i < 12; ++i)
    {
        MoveToEx(hdc, centerX + vertices[lines[i][0]].x, centerY + vertices[lines[i][0]].y, NULL);
        LineTo(hdc, centerX + vertices[lines[i][1]].x, centerY + vertices[lines[i][1]].y);
    }

    // Define the origin and axis endpoints
    Point origin(0, 0, 0);
    Point xAxis(size * 2, 0, 0);
    Point yAxis(0, size * 2, 0);
    Point zAxis(0, 0, size * 2);

    // Rotate the axis endpoints
    xAxis = RotatePoint(xAxis, angleX/16, angleY/16, angleZ/16);
    yAxis = RotatePoint(yAxis, angleX/16, angleY/16, angleZ/16);
    zAxis = RotatePoint(zAxis, angleX/16, angleY/16, angleZ/16);

    // Draw the axes
    HPEN redPen = CreatePen(PS_SOLID, 2, RGB(0, 0, 255)); // X-axis in red
    HPEN greenPen = CreatePen(PS_SOLID, 2, RGB(0, 255, 0)); // Y-axis in green
    HPEN bluePen = CreatePen(PS_SOLID, 2, RGB(255, 0, 0)); // Z-axis in blue

    HPEN oldPen = (HPEN)SelectObject(hdc, redPen);
    MoveToEx(hdc, centerX + origin.x, centerY + origin.y, NULL);
    LineTo(hdc, centerX + xAxis.x, centerY + xAxis.y);

    SelectObject(hdc, greenPen);
    MoveToEx(hdc, centerX + origin.x, centerY + origin.y, NULL);
    LineTo(hdc, centerX + yAxis.x, centerY + yAxis.y);

    SelectObject(hdc, bluePen);
    MoveToEx(hdc, centerX + origin.x, centerY + origin.y, NULL);
    LineTo(hdc, centerX + zAxis.x, centerY + zAxis.y);

    // Restore the old pen and delete created pens
    SelectObject(hdc, oldPen);
    DeleteObject(redPen);
    DeleteObject(greenPen);
    DeleteObject(bluePen);
}

KdTree Q;
//std::vector <Point> points = {Point(111, 110), Point(60, 205), Point(80, 64), Point(122, 135), Point(158, 128)};


std::vector<Point> points =
{
//    Point(-30, +30, +20),
//    Point(-100, 100, 100),
    Point(-70, 50, 50),
//    Point(75, -50, 50),
    Point(30, -70, 20),
    Point(40, -35 ,0),
    Point(20, -80 , 75),
    Point(0, 75, 0)


//    Point(-30, +30, +20),

//    Point(-100, 100, 100),

//    Point(-70, 50, 50),
//    Point(75, -50, 50),
//    Point(30, -70, 20),
//    Point(40, -35 ,0),

//    Point(20, -80 , 75),
//    Point(0, 0, 0)
};

std::vector <Point> nearests;
int pxi, pyi, pxf, pyf;
Point nearest;
Point target;
int n_points;
double angle1 = 0;
double angle2 = 0;
double angle3 = 0;


BOOL CALLBACK DlgMain(HWND hwndDlg, UINT uMsg, WPARAM wParam, LPARAM lParam)
{

    switch(uMsg)
    {
    case WM_INITDIALOG:
    {
//        Q.build(points);


        pxi=10,pyi=50,pxf=210,pyf=250;

//        SendMessage(GetDlgCtrlID(IDC_SLIDER_1), TBM_SETPOS, TRUE, 50); // Valor inicial 50
//        SendMessage(GetDlgCtrlID(IDC_SLIDER_2), TBM_SETPOS, TRUE, 75); // Valor inicial 75

        HWND hSlider1 = GetDlgItem(hwndDlg, IDC_SLIDER_1);
    HWND hSlider2 = GetDlgItem(hwndDlg, IDC_SLIDER_2);

    // Establecer el rango del scroll bar (0 a 100 por ejemplo)
    SendMessage(hSlider1, TBM_SETRANGE, TRUE, MAKELONG(0, 100));
    SendMessage(hSlider2, TBM_SETRANGE, TRUE, MAKELONG(0, 100));

    // Establecer la posición inicial del scroll bar
    SendMessage(hSlider1, TBM_SETPOS, TRUE, 50); // Valor inicial 50
    SendMessage(hSlider2, TBM_SETPOS, TRUE, 50); // Valor inicial 75
    }
    return TRUE;
    case WM_PAINT:
    {
        PAINTSTRUCT ps;
        HDC hdc;
        hdc=BeginPaint(hwndDlg,&ps);
                Q.build(points);
        Q.MostrarR(hdc, 800, 60, 120);
        Q.Mostrar(hdc, 200, 240, angle1, angle2, angle3);//        if ( n_points > 1) {


//        char Cad[10];
//        sprintf(Cad,"%i",angle1);
//        TextOut(hdc,100,100,Cad,strlen(Cad));
//
//        char Cad2[10];
//        sprintf(Cad2,"%i",angle2);
//        TextOut(hdc,100,130,Cad2,strlen(Cad2));


        RECT clientRect;
        GetClientRect(hwndDlg, &clientRect);
        int centerX = (clientRect.right - clientRect.left) / 2;
        int centerY = (clientRect.bottom - clientRect.top) / 2;
        int size = 100; // Adjust size as needed
        DrawCube(hdc, 200, 240, size, angle1, angle2, angle3);



        EndPaint(hwndDlg,&ps);
    }
    return TRUE;
    case WM_MOUSEMOVE:
    {
        long xPos = GET_X_LPARAM(lParam);
        long yPos = GET_Y_LPARAM(lParam);
        SetDlgItemInt(hwndDlg,EDITX,(int)xPos, FALSE);
        SetDlgItemInt(hwndDlg,EDITY,(int)yPos, FALSE);
    }
    return TRUE;
    case WM_LBUTTONDOWN:
    {
        long xPos = GET_X_LPARAM(lParam);
        long yPos = GET_Y_LPARAM(lParam);
        if(xPos<pxf&&xPos>pxi&&yPos<pyf&&yPos>pyi)
        {

            Q.insertion(Point(xPos, yPos));

        }
        InvalidateRect(hwndDlg,NULL,true);
        UpdateWindow(hwndDlg); // Add this line to update the window

    }
    return TRUE;
    case WM_CLOSE:
    {
        EndDialog(hwndDlg, 0);
    }
    return TRUE;
    case WM_COMMAND:
    {
        switch(LOWORD(wParam))
        {
        case BUTTON1:
//            int xxas =0, yyas=0, zzas=0;
//
//            xxas =GetDlgItemInt(hwndDlg,EDITXX, 0, FALSE);
//            yyas =GetDlgItemInt(hwndDlg,EDITYY, 0, FALSE);
//            zzas =GetDlgItemInt(hwndDlg,EDITN, 0, FALSE);




                int xxas = 0, yyas = 0, zzas = 0;
                char buffer[256];

                // Obtener el valor del primer punto y convertirlo a entero
                GetDlgItemText(hwndDlg, EDITXX, buffer, sizeof(buffer));
                xxas = atoi(buffer);

                // Obtener el valor del segundo punto y convertirlo a entero
                GetDlgItemText(hwndDlg, EDITYY, buffer, sizeof(buffer));
                yyas = atoi(buffer);

                // Obtener el valor del tercer punto y convertirlo a entero
                GetDlgItemText(hwndDlg, EDITN, buffer, sizeof(buffer));
                zzas = atoi(buffer);

//            n_points = GetDlgItemInt(hwndDlg,EDITN, 0, FALSE);
//
//            target = Point(xxas, yyas);
//            if ( n_points > 1)
//            {
//
//                nearests = Q.nearest_points(Point(xxas, yyas), n_points);
//            }
//            else
//            {
//                nearest = Q.nearest_point(Point(xxas, yyas));
//            }
//            std::cout << "Nearest neighbors to (" << target.x << ", " << target.y << "):\n";
//            for (const auto& point : nearests)
//            {
//                std::cout << "(" << point.x << ", " << point.y << ")\n";
//            }
//            std::cout << '\n';

            points.push_back(Point(xxas, yyas, zzas));
            InvalidateRect(hwndDlg,NULL,true);

            return TRUE;
        }
    }
    return TRUE;
    case WM_HSCROLL:
{
    HWND hScrollBar = (HWND)lParam;
    int controlId = GetDlgCtrlID(hScrollBar);
    int scrollPos = SendMessage(hScrollBar, TBM_GETPOS, 0, 0);

    SetDlgItemInt(hwndDlg,EDIT1,(int)controlId, FALSE);
//    SetDlgItemInt(hwndDlg,EDIT2,(int)controlId, FALSE);
    switch (controlId)
    {
    case IDC_SLIDER_1: // X-axis rotation
        angle1 = 70 + ( scrollPos) * (2 * M_PI) / 4;
        SetDlgItemInt(hwndDlg,EDIT1,(int)angle1, FALSE);

        break;
    case IDC_SLIDER_2: // Y-axis rotation
        angle2 =  125 + (scrollPos) * (2 * M_PI) / 4;
                SetDlgItemInt(hwndDlg,EDIT2,(int)angle2, FALSE);

        break;
    case IDC_SLIDER_3: // Z-axis rotation
        angle3 = scrollPos * (2 * M_PI) / 4;
                SetDlgItemInt(hwndDlg,EDIT3,(int)angle3, FALSE);

        break;
    default:
        break;
    }

    InvalidateRect(hwndDlg, NULL, TRUE);
}
    return TRUE;

    case WM_VSCROLL:
{
    HWND hScrollBar = (HWND)lParam;
    int controlId = GetDlgCtrlID(hScrollBar);
    int scrollPos = SendMessage(hScrollBar, TBM_GETPOS, 0, 0);

    SetDlgItemInt(hwndDlg,EDIT1,(int)controlId, FALSE);
//    SetDlgItemInt(hwndDlg,EDIT2,(int)controlId, FALSE);
    switch (controlId)
    {
    case IDC_SLIDER_1: // X-axis rotation
        angle1 = 70 + ( scrollPos) * (2 * M_PI) / 4;
        SetDlgItemInt(hwndDlg,EDIT1,(int)angle1, FALSE);

        break;
    case IDC_SLIDER_2: // Y-axis rotation
        angle2 =  125 + (scrollPos) * (2 * M_PI) / 4;
                SetDlgItemInt(hwndDlg,EDIT2,(int)angle2, FALSE);

        break;
    case IDC_SLIDER_3: // Z-axis rotation
        angle3 = scrollPos * (2 * M_PI) / 4;
                SetDlgItemInt(hwndDlg,EDIT3,(int)angle3, FALSE);

        break;
    default:
        break;
    }

    InvalidateRect(hwndDlg, NULL, TRUE);
}
    return TRUE;



    }
    return FALSE;
}

int APIENTRY WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nShowCmd)
{


    ULONG_PTR gdiplusToken; //1
    InitGDIPlus(&gdiplusToken);//2



    hInst=hInstance;
    InitCommonControls();
    return DialogBox(hInst, MAKEINTRESOURCE(DLG_MAIN), NULL, (DLGPROC)DlgMain);



    ShutdownGDIPlus(gdiplusToken);// 3

}
