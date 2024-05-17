#ifndef CUSTOMBUTTON_H
#define CUSTOMBUTTON_H

#include <QPushButton>
#include <QMouseEvent>

class CustomButton : public QPushButton
{
    Q_OBJECT

public:
    explicit CustomButton(QWidget *parent = nullptr);

signals:
    void customClicked(const QString &flag);  // Custom signal with flag

protected:
    void mousePressEvent(QMouseEvent *event) override;
};
#endif // CUSTOMBUTTON_H
