#include <windows.h>
#include <commctrl.h>
#include <stdio.h>
#include "resource.h"
#include <string>
#include <vector>
#include <math.h>

HINSTANCE hInst;

using namespace std;

int imageWidth = 0;
int imageHeight = 0;

//HBITMAP hBitmap = NULL;
HBITMAP hLoadedBitmap = NULL;
std::vector<std::vector<COLORREF>> imagePixels; // To store pixel values as colors


void showImage(HDC hdc, HBITMAP hBitmap, int x, int y)
{
    if (hBitmap != NULL)
    {
        HDC hdcMem = CreateCompatibleDC(hdc);
        HBITMAP hBitmapOld = (HBITMAP)SelectObject(hdcMem, hBitmap);
        BITMAP bm;
        GetObject(hBitmap, sizeof(bm), &bm);
        BitBlt(hdc, x, y, bm.bmWidth, bm.bmHeight, hdcMem, 0, 0, SRCCOPY);
        SelectObject(hdcMem, hBitmapOld);
        DeleteDC(hdcMem);
    }
}

void loadPixels(HBITMAP hBitmap, int x, int y, int width, int height)
{
    imagePixels.clear();
    HDC hdcBitmap = CreateCompatibleDC(NULL);
    SelectObject(hdcBitmap, hBitmap);

    for (int j = y; j < y + height; j++)
    {
        std::vector<COLORREF> row;
        for (int i = x; i < x + width; i++)
        {
            COLORREF color = GetPixel(hdcBitmap, i, j);
            row.push_back(color);
        }
        imagePixels.push_back(row);
    }

    DeleteDC(hdcBitmap);
}




void loadPixelC(HDC hdc, int x, int y, int t, int mx, int my,int tt)
{
    StretchBlt(hdc, mx,my, tt,tt,hdc, x,y,t,t, SRCCOPY);
}




bool LoadImageFile(HWND hwndDlg, LPCSTR filePath)
{
    hLoadedBitmap = (HBITMAP)LoadImage(NULL, filePath, IMAGE_BITMAP, 0, 0, LR_LOADFROMFILE | LR_CREATEDIBSECTION);
    if (hLoadedBitmap == NULL)
    {
        DWORD dwError = GetLastError();
        LPSTR lpMsgBuf;
        FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
                       NULL, dwError, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&lpMsgBuf, 0, NULL);
        MessageBoxA(hwndDlg, lpMsgBuf, "Error", MB_OK | MB_ICONERROR);
        LocalFree(lpMsgBuf);
        return false;
    }

    BITMAP bm;
    GetObject(hLoadedBitmap, sizeof(BITMAP), &bm);
    imageWidth = bm.bmWidth;
    imageHeight = bm.bmHeight;

    // Get pixel data
    HDC hdc = GetDC(NULL);
    HDC hdcMem = CreateCompatibleDC(hdc);
    SelectObject(hdcMem, hLoadedBitmap);

    imagePixels.clear();
    for (int y = 0; y < imageHeight; ++y)
    {
        std::vector<COLORREF> row;
        for (int x = 0; x < imageWidth; ++x)
        {
            COLORREF color = GetPixel(hdcMem, x, y);
            row.push_back(color);
        }
        imagePixels.push_back(row);
    }

    ReleaseDC(NULL, hdc);
    DeleteDC(hdcMem);

    return true;
}

class Node
{
public:
    RECT region;
    bool isLeaf;
    Node* children[4]; // Assuming 4 children (NW, NE, SW, SE)
    COLORREF pixelValue;

    Node(RECT rect = {}, bool leaf = false, Node* nw = nullptr, Node* ne = nullptr, Node* sw = nullptr, Node* se = nullptr)
    {
        region = rect;
        isLeaf = leaf;
        children[0] = nw;
        children[1] = ne;
        children[2] = sw;
        children[3] = se;
    }
};


class QuadTree
{
public:
    Node* root;
    QuadTree()
    {
        root = NULL;
    }
    bool IsHomogeneous(const std::vector<std::vector<COLORREF>>& pixels, int x, int y, int width, int height, int threshold)
    {
        COLORREF targetPixel = pixels[y][x];
        for (int j = y; j < y + height; ++j)
        {
            for (int i = x; i < x + width; ++i)
            {
                if (abs(GetRValue(pixels[j][i]) - GetRValue(targetPixel)) > threshold ||
                        abs(GetGValue(pixels[j][i]) - GetGValue(targetPixel)) > threshold ||
                        abs(GetBValue(pixels[j][i]) - GetBValue(targetPixel)) > threshold)
                {
                    //promediar
                    return false;
                }
            }
        }
        return true;
    }
    bool IsHomogeneous2(const std::vector<std::vector<COLORREF>>& pixels, int x, int y, int width, int height)
    {
        COLORREF sumColor = 0;
        int count = 0;

        for (int j = y; j < y + height; ++j)
        {
            for (int i = x; i < x + width; ++i)
            {
                sumColor += pixels[j][i];
                ++count;
            }
        }
        COLORREF averageColor = sumColor / count;

        for (int j = y; j < y + height; ++j)
        {
            for (int i = x; i < x + width; ++i)
            {
                if (pixels[j][i] != averageColor)
                {
                    return false;
                }
            }
        }

        return true;
    }

    double CalculateStandardDeviation(const std::vector<std::vector<COLORREF>>& pixels, int x, int y, int width, int height)
    {
        double sumR = 0, sumG = 0, sumB = 0, sumSqR = 0, sumSqG = 0, sumSqB = 0;
        int count = 0;

        // Calculate the sum of color values and the sum of squared color values
        for (int j = y; j < y + height; ++j)
        {
            for (int i = x; i < x + width; ++i)
            {
                COLORREF color = pixels[j][i];
                sumR += GetRValue(color);
                sumG += GetGValue(color);
                sumB += GetBValue(color);
                sumSqR += GetRValue(color) * GetRValue(color);
                sumSqG += GetGValue(color) * GetGValue(color);
                sumSqB += GetBValue(color) * GetBValue(color);
                ++count;
            }
        }

        // Calculate the mean of color values
        double meanR = sumR / count;
        double meanG = sumG / count;
        double meanB = sumB / count;

        // Calculate the standard deviation of color values
        double stdDevR = sqrt((sumSqR / count) - (meanR * meanR));
        double stdDevG = sqrt((sumSqG / count) - (meanG * meanG));
        double stdDevB = sqrt((sumSqB / count) - (meanB * meanB));

        // Calculate the overall standard deviation
        double stdDev = (stdDevR + stdDevG + stdDevB) / 3.0;

        return stdDev;
    }

// Function to determine homogeneity based on standard deviation
    bool IsHomogeneousByStdDev(const std::vector<std::vector<COLORREF>>& pixels, int x, int y, int width, int height, double threshold)
    {
        // Calculate the standard deviation of color values within the region
        double stdDev = CalculateStandardDeviation(pixels, x, y, width, height);

        // Compare the standard deviation to the threshold
        return stdDev <= threshold;
    }


    bool IsHomogeneous3(const std::vector<std::vector<COLORREF>>& pixels, int x, int y, int width, int height, int threshold)
    {
        COLORREF sumColor = 0;
        int count = 0;

        // Calculate the sum of colors in the region
        for (int j = y; j < y + height; ++j)
        {
            for (int i = x; i < x + width; ++i)
            {
                sumColor += pixels[j][i];
                ++count;
            }
        }

        // Calculate the average color
        COLORREF averageColor = sumColor / count;

        // Define a threshold for color difference
//    int threshold = 32; // Adjust this threshold as needed

        // Compare each pixel's color to the average color with a threshold
        for (int j = y; j < y + height; ++j)
        {
            for (int i = x; i < x + width; ++i)
            {
                // Calculate the absolute difference between pixel color and average color
                int diffR = abs(GetRValue(pixels[j][i]) - GetRValue(averageColor));
                int diffG = abs(GetGValue(pixels[j][i]) - GetGValue(averageColor));
                int diffB = abs(GetBValue(pixels[j][i]) - GetBValue(averageColor));

                // Check if the difference exceeds the threshold
                if (diffR > threshold || diffG > threshold || diffB > threshold)
                {
                    return false;
                }
            }
        }

        return true;
    }



    Node* CreateQuadtree(const std::vector<std::vector<COLORREF>>& pixels, int x, int y, int width, int height, int nodos, int color_diff)
    {

        if (width <= nodos || height <= nodos || IsHomogeneousByStdDev(pixels, x, y, width, height, color_diff)) {
            double sumRed = 0.0, sumGreen = 0.0, sumBlue = 0.0;
            int count = 0;

            // Calculate the sum of colors in the region
            for (int j = y; j < y + height; ++j) {
                for (int i = x; i < x + width; ++i) {
                    sumRed += GetRValue(pixels[j][i]) / 255.0;
                    sumGreen += GetGValue(pixels[j][i]) / 255.0;
                    sumBlue += GetBValue(pixels[j][i]) / 255.0;
                    ++count;
                }
            }

            // Calculate the average color
            COLORREF averageColor = RGB(static_cast<int>(sumRed / count * 255),
                                        static_cast<int>(sumGreen / count * 255),
                                        static_cast<int>(sumBlue / count * 255));

            Node* leaf = new Node();
            SetRect(&leaf->region, x, y, x + width, y + height);
            leaf->isLeaf = true;
            leaf->pixelValue = averageColor;
            return leaf;
        }


        // if (width <= nodos || height <= nodos || IsHomogeneous(pixels, x, y, width, height, color_diff))
        // {
        //     Node* leaf = new Node();
        //     SetRect(&leaf->region, x, y, x + width, y + height);
        //     leaf->isLeaf = true;
        //     leaf->pixelValue = pixels[y][x]; // Store pixel value at top-left corner for simplicity
        //     return leaf;
        // }



        Node* node = new Node();
        SetRect(&node->region, x, y, x + width, y + height);
        node->isLeaf = false;
        int subWidth = width / 2;
        int subHeight = height / 2;
        node->children[0] = CreateQuadtree(pixels, x, y, subWidth, subHeight, nodos, color_diff); // NW quadrant
        node->children[1] = CreateQuadtree(pixels, x + subWidth, y, width - subWidth, subHeight, nodos, color_diff); // NE quadrant
        node->children[2] = CreateQuadtree(pixels, x, y + subHeight, subWidth, height - subHeight, nodos, color_diff); // SW quadrant
        node->children[3] = CreateQuadtree(pixels, x + subWidth, y + subHeight, width - subWidth, height - subHeight, nodos, color_diff); // SE quadrant
        return node;
    }


    void DrawQuadtree(HDC hdc, Node* node, bool lines, bool color)
    {
        if (node == nullptr)
        {
            return;
        }

        if (node->isLeaf)
        {
            if (color)
            {
                HBRUSH hBrush = CreateSolidBrush(node->pixelValue);
                FillRect(hdc, &node->region, hBrush);
                DeleteObject(hBrush);
            }
        }
        else
        {
            for (int i = 0; i < 4; ++i)
            {
                DrawQuadtree(hdc, node->children[i], lines, color);
            }
        }

        if (lines)
        {
            HPEN hGridPen = CreatePen(PS_SOLID, 1, RGB(120, 0, 120));
            SelectObject(hdc, hGridPen);
            MoveToEx(hdc, node->region.left, node->region.top, NULL);
            LineTo(hdc, node->region.left, node->region.bottom);
            MoveToEx(hdc, node->region.left, node->region.top, NULL);
            LineTo(hdc, node->region.right, node->region.top);

            DeleteObject(hGridPen);
        }


    }

};

int n_node_side = 256;
int diff_color = 16;
bool lineas = 1;
bool colorr = 1;
HWND hSliderColor;
HWND hSliderNodeSide;

int CalculateColorDifference(int pos)
{
    if (pos == 0)
        return 1;
    else
        return 1 << (pos - 1);
}
// Función para calcular el tamaño de nodo correspondiente a partir de la posición del control deslizante
int CalculateNodeSize(int pos)
{
    if (pos == 0)
        return 1;
    else
        return 1 << (pos - 1);
}

BOOL CALLBACK DlgMain(HWND hwndDlg, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    static string imagen = "nose.bmp";
    switch(uMsg)
    {
    case WM_INITDIALOG:
    {

        // Configurar el control deslizante para el tamaño de nodo
        hSliderNodeSide = GetDlgItem(hwndDlg, IDC_SLIDER_NODE_SIDE);
        SendMessage(hSliderNodeSide, TBM_SETRANGE, TRUE, MAKELONG(0, 9)); // Rango de 0 a 8
        SendMessage(hSliderNodeSide, TBM_SETPAGESIZE, 0, 1); // Tamaño de página de 1
        SendMessage(hSliderNodeSide, TBM_SETTICFREQ, 1, 0); // Frecuencia de las marcas de 1

        int values[9] = {1, 2, 4, 8, 16, 32, 64, 128, 256};
        for (int i = 1; i < 10; ++i)
        {
            SendMessage(hSliderNodeSide, TBM_SETPOS, TRUE, values[i]);
        }

        hSliderColor = GetDlgItem(hwndDlg, IDC_SLIDER_COLOR);
        SendMessage(hSliderColor, TBM_SETRANGE, TRUE, MAKELONG(0, 9)); // Rango de 0 a 7 (para 2^7 = 128)
        SendMessage(hSliderColor, TBM_SETPAGESIZE, 0, 1); // Tamaño de página de 1
        SendMessage(hSliderColor, TBM_SETTICFREQ, 1, 0); // Frecuencia de las marcas de 1

        int values2[9] = {1, 2, 4, 8, 16, 32, 64, 128, 256};
        for (int i = 1; i < 10; ++i)
        {
            SendMessage(hSliderNodeSide, TBM_SETPOS, TRUE, values2[i]);
        }



        if (!LoadImageFile(hwndDlg, "logo.bmp"))
        {
            EndDialog(hwndDlg, 0);
        }
        return TRUE; // Indicate that the initialization was successful
    }

    return TRUE;
    case WM_PAINT:
    {
        HDC hdc;
        PAINTSTRUCT ps;
        hdc = BeginPaint(hwndDlg, &ps);
//        Rectangle(hdc, 10, 10, 270, 270);
//        showImage(hdc, hLoadedBitmap, 510, 10);
//        loadPixelC(hdc, 510, 10, 256, 766, 10, 50);


        QuadTree QT;
        Node* root = QT.CreateQuadtree(imagePixels, 0, 0, imageWidth, imageHeight, imageHeight / n_node_side, diff_color);

        QT.DrawQuadtree(hdc, root, lineas, colorr);

        EndPaint(hwndDlg, &ps);

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
            n_node_side =GetDlgItemInt(hwndDlg,EDIT_C, 0, FALSE);
            diff_color =GetDlgItemInt(hwndDlg,EDIT_N, 0, FALSE);

            InvalidateRect(hwndDlg,NULL,true);
            return TRUE;
        case BUTTON2:
            lineas = !lineas;
            InvalidateRect(hwndDlg,NULL,true);

            return TRUE;
        case BUTTON3:
            colorr = !colorr;
            InvalidateRect(hwndDlg,NULL,true);

            return TRUE;
        }
    }
    return TRUE;

    case WM_HSCROLL:
    {
        if ((HWND)lParam == hSliderColor || (HWND)lParam == hSliderNodeSide)
        {
            int value = SendMessage((HWND)lParam, TBM_GETPOS, 0, 0);

            if ((HWND)lParam == hSliderColor)
                diff_color = CalculateColorDifference(value);
            else if ((HWND)lParam == hSliderNodeSide)
                n_node_side = CalculateNodeSize(value);

            InvalidateRect(hwndDlg, NULL, TRUE);
        }
        return TRUE;
    }
    }
    return FALSE;
}


int APIENTRY WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nShowCmd)
{
    hInst=hInstance;
    InitCommonControls();
    return DialogBox(hInst, MAKEINTRESOURCE(DLG_MAIN), NULL, (DLGPROC)DlgMain);
}


