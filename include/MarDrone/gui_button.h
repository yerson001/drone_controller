#ifndef GUI_BUTTON_H
#define GUI_BUTTON_H
#include <opencv2/opencv.hpp>

class gui_button {
private:
    cv::Rect rect;
    std::string label;
    cv::Scalar color;
    bool clicked;

public:
    gui_button(int x, int y, int width, int height, const std::string& labelText, const cv::Scalar& buttonColor)
        : rect(x, y, width, height), label(labelText), color(buttonColor), clicked(false) {}

    void draw(cv::Mat& image) const {
        cv::rectangle(image, rect, color, -1);
        cv::putText(image, label, cv::Point(rect.x + 10, rect.y + rect.height/2 + 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 2);
    }

    bool isClicked(int x, int y) {
        return rect.contains(cv::Point(x, y));
    }

    void setClicked(bool state) {
        clicked = state;
    }

    bool isClicked() const {
        return clicked;
    }
};
#endif
