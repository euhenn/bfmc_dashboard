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
    QString flag;

    if (x < width / 2 && y < height / 2)
    {
        qDebug() << "Left top";
        flag = "--leftup";
    }
    else if (x >= width / 2 && y < height / 2)
    {
        qDebug() << "Right top";
        flag = "--rightup";
    }
    else if (x < width / 2 && y >= height / 2)
    {
        qDebug() << "Left bottom";
        flag = "--leftdown";
    }
    else if (x >= width / 2 && y >= height / 2)
    {
        qDebug() << "Right bottom";
        flag = "--rightdown";
    }

    emit customClicked(flag);  // Emit custom signal with flag

    // Call the base class implementation
    QPushButton::mousePressEvent(event);
}
