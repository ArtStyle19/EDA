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

HINSTANCE hInst;
class Point {
public:
    int x, y;
    Point(int x = 0, int y = 0) : x(x), y(y) {
    }
};

class Node {
public:
    int depth;
    Point main_point;
    Node* children[2];
    Node(Point main, int depth = 0) : depth(depth) {
        main_point = main;
        children[0] = nullptr;
        children[1] = nullptr;
    }
};

class Comparison {
public:
    int axis;
    Comparison (int axis = 0) : axis(axis) {};
    bool operator() (const Point &p1, const Point &p2) {
        return (axis == 0) ? p1.x < p2.x : p1.y < p2.y;
    }
};
class KdTree {
public:
    int k = 2;
    Node* root;
    KdTree() {
        root = nullptr;
    }

    void build(std::vector <Point> &points) {
        build_helper(points, root, 0);
    }
    void build_helper(std::vector <Point> &points, Node* &curr_node, int depth) {
        if (points.empty()) {
            return;
        }

        int axis = depth % k;
        std::sort(points.begin(), points.end(), Comparison(axis));

        int median_index = points.size() / 2;
        curr_node = new Node(points[median_index], depth);

        std::vector <Point> left(points.begin(), points.begin() + median_index);
        std::vector <Point> right(points.begin() + median_index + 1, points.end());

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
        int median_index = points.size() / 2;\
        std::sort(points.begin(), points.end(), Comparison(axis));

        curr_node = new Node(points[median_index], depth);

        std::vector<Point> left(points.begin(), points.begin() + median_index);
        balanceHelper(curr_node->children[0], left, depth + 1);

        std::vector<Point> right(points.begin() + median_index + 1, points.end());
        balanceHelper(curr_node->children[1], right, depth + 1);
    }



    //////////// dx

    struct PointDistance {
        Point point;
        float distance;

        PointDistance(Point p, float d) : point(p), distance(d) {}

        // Overload < operator for priority queue
        bool operator<(const PointDistance& other) const {
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

    Point nearest_point(Point pivot) {
        return nearest_point_helper(pivot, root, 0);
    }
    float dis(const Point &p1, const Point &p2) {
        return sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y)) ;
    }

    Point nearest_point_helper(Point pivot, Node* curr_node, int depth) {
        if (curr_node == nullptr) {
            return Point(1000, 1000);
        }

        int axis = depth % k;

        Point best = curr_node->main_point;
        Node* good_side = nullptr;
        Node* bad_side = nullptr;

        if (axis == 0) {
            if (pivot.x < curr_node->main_point.x) {
                good_side = curr_node->children[0];
                bad_side = curr_node->children[1];
            } else {
                good_side = curr_node->children[1];
                bad_side = curr_node->children[0];
            }
        } else {
            if (pivot.y < curr_node->main_point.y) {
                good_side = curr_node->children[0];
                bad_side = curr_node->children[1];
            } else {
                good_side = curr_node->children[1];
                bad_side = curr_node->children[0];
            }
        }

        Point best_good_side = nearest_point_helper(pivot, good_side, depth + 1);

        if (best.x  == INT_MAX || best_good_side.x == INT_MAX) {
            return curr_node->main_point;
        }
        if (dis(pivot, best) > dis(pivot, best_good_side)) {
            best = best_good_side;
        }
        if (is_backtrack_needed(curr_node, pivot, best, depth)) {
            best = closest(pivot, best, nearest_point_helper(pivot, bad_side, depth + 1));
        }
        return best;
    }

    Point closest(Point pivot, Point p1, Point p2) {
        if (dis(pivot, p1) < dis(pivot, p2)) {
            return p1;
        } else {
            return p2;
        }
    }

    bool is_backtrack_needed(Node* curr_node, Point pivot, Point best, int depth) {
        if (curr_node == nullptr) return false;
        int axis = depth % k;
        if (axis == 0) {
            if (curr_node->main_point.x > pivot.x && curr_node->main_point.x - pivot.x < dis(pivot, best))
                return true;

            if (curr_node->main_point.x < pivot.x && pivot.x - curr_node->main_point.x < dis(pivot, best))
                return true;
        } else {

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
            sprintf(Cad,"%i",R->depth % 2);
            TextOut(hdc,x,y+30,Cad,strlen(Cad));
            MosR(R->children[0],hdc,x-a,y+50,a/2);
            MosR(R->children[1],hdc,x+a,y+50,a/2);
        }
    }
    void Mostrar(HDC hdc,int xi,int yi,int xf,int yf)
    {
        Mos(root,hdc,xi,yi,xf,yf);
    }
    void Mos(Node *R,HDC hdc,int xi,int yi,int xf,int yf)
    {
        if(R!=NULL)
        {
            int xx=R->main_point.x;
            int yy=R->main_point.y;
            if((R->depth % k )==1)
            {
                MoveToEx(hdc,xi,yy,NULL);
                LineTo(hdc,xf,yy);
                Mos(R->children[0],hdc,xi,yi,xf,yy);
                Mos(R->children[1],hdc,xi,yy,xf,yf);
            }
            else
            {
                MoveToEx(hdc,xx,yi,NULL);
                LineTo(hdc,xx,yf);
                Mos(R->children[0],hdc,xi,yi,xx,yf);
                Mos(R->children[1],hdc,xx,yi,xf,yf);

            }
            Ellipse(hdc,xx-4,yy-4,xx+4,yy+4);
        }
    }
};

void DrawNumber(HWND hwnd, HDC hdc)
{
    char cad[100];
    strcpy(cad, "100"); // Almacenar "100" en el array de caracteres
    SetTextColor(hdc, RGB(0, 0, 0));
    // Configurar el modo de dibujo
    SetBkMode(hdc, TRANSPARENT); // Texto sin fondo
    // Dibujar el número "100" en el centro de la ventana
    TextOutA(hdc, 100, 100, cad, strlen(cad)); // Cambia las coordenadas según sea necesario
}

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

    int diffx = abs(n.x - target.x);
        int diffy = abs(n.y - target.y);
        int radius = static_cast<int>(std::sqrt(diffx * diffx + diffy * diffy)); // Distancia euclidiana

//         Dibuja la circunferencia de la elipse centrada en el punto objetivo con el mismo radio
        Arc(hdc, target.x - radius - 2, target.y - radius - 2, target.x + radius + 2, target.y + radius + 2, 0, 0, 0, 0);

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

//         Calcula el radio basado en la distancia entre el punto y el objetivo
        int diffx = abs(point.x - target.x);
        int diffy = abs(point.y - target.y);
        int radius = static_cast<int>(std::sqrt(diffx * diffx + diffy * diffy)); // Distancia euclidiana

//         Dibuja la circunferencia de la elipse centrada en el punto objetivo con el mismo radio
        Arc(hdc, target.x - radius - 2, target.y - radius - 2, target.x + radius + 2, target.y + radius + 2, 0, 0, 0, 0);
    }
    DeleteObject(hEllipsePen);
    DeleteObject(hEllipsePen2);
    DeleteObject(hEllipseBrush);
}


KdTree Q;
std::vector <Point> points = {Point(111, 110), Point(60, 205), Point(80, 64), Point(122, 135), Point(158, 128)};
std::vector <Point> nearests;
int pxi, pyi, pxf, pyf;
Point nearest;
Point target;
int n_points;

BOOL CALLBACK DlgMain(HWND hwndDlg, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    switch(uMsg)
    {
    case WM_INITDIALOG:
    {
        Q.build(points);


        pxi=10,pyi=50,pxf=210,pyf=250;

    }
    return TRUE;
    case WM_PAINT:
    {
        PAINTSTRUCT ps;
        HDC hdc;
        hdc=BeginPaint(hwndDlg,&ps);
        Q.MostrarR(hdc,750,50,200);
        Rectangle(hdc,pxi,pyi,pxf,pyf);

        Q.Mostrar(hdc,pxi,pyi,pxf,pyf);
        if ( n_points > 1) {
            MostrarCercanos(hdc, nearests, target);
        } else {
            MostrarCercano(hdc, nearest, target);
        }

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
//            points.push_back(Point(xPos, yPos));
//            Q.build(points);
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
            int xxas =0 , yyas=0;

            xxas =GetDlgItemInt(hwndDlg,EDITXX, 0, FALSE);
            yyas =GetDlgItemInt(hwndDlg,EDITYY, 0, FALSE);

            n_points = GetDlgItemInt(hwndDlg,EDITN, 0, FALSE);

            target = Point(xxas, yyas);
            if ( n_points > 1) {
                nearests = Q.nearest_points(Point(xxas, yyas), n_points);\
            } else {
                nearest = Q.nearest_point(Point(xxas, yyas));
            }
            std::cout << "Nearest neighbors to (" << target.x << ", " << target.y << "):\n";
            for (const auto& point : nearests)
            {
                std::cout << "(" << point.x << ", " << point.y << ")\n";
            }
            std::cout << '\n';

            InvalidateRect(hwndDlg,NULL,true);

            return TRUE;
        }
    }
    return TRUE;
    }
    return FALSE;
}

int APIENTRY WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nShowCmd)
{
    hInst=hInstance;
    InitCommonControls();
    return DialogBox(hInst, MAKEINTRESOURCE(DLG_MAIN), NULL, (DLGPROC)DlgMain);
}
