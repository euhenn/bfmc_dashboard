#include "custombutton.h"
#include <QDebug>

CustomButton::CustomButton(QWidget *parent) : QPushButton(parent)
{
}

void CustomButton::mousePressEvent(QMouseEvent *event)
{
    int x = event->pos().x();
    int y = event->pos().y();
    int width = this->width();
    int height = this->height();

    if (x < width / 2 && y < height / 2)
    {
        //qDebug() << "Left top";
        flag = "--north";
    }
    else if (x >= width / 2 && y < height / 2)
    {
        //qDebug() << "Right top";
        flag = "--south";
    }
    else if (x < width / 2 && y >= height / 2)
    {
        //qDebug() << "Left bottom";
        flag = "--west";
    }
    else if (x >= width / 2 && y >= height / 2)
    {
        //qDebug() << "Right bottom";
        flag = "--east";
    }

    // Call the base class implementation
    QPushButton::mousePressEvent(event);
}

QString CustomButton::getFlag() const
{
    return flag;
}
