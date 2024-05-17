#ifndef CUSTOMBUTTON_H
#define CUSTOMBUTTON_H

#include <QPushButton>
#include <QMouseEvent>

class CustomButton : public QPushButton
{
    Q_OBJECT

public:
    explicit CustomButton(QWidget *parent = nullptr);
    QString getFlag() const;  // Method to get the flag

protected:
    void mousePressEvent(QMouseEvent *event) override;

private:
    QString flag;  // Store the flag based on the last click position
};

#endif // CUSTOMBUTTON_H
