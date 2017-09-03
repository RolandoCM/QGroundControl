#ifndef QGCHILCONDORCONFIGURATION_H
#define QGCHILCONDORCONFIGURATION_H

#include <QWidget>

#include "QGCHilLink.h"

class QGCHilConfiguration;
namespace Ui {
class QGCHilCondorConfiguration;
}

class QGCHilCondorConfiguration : public QWidget
{
    Q_OBJECT
    
public:
    explicit QGCHilCondorConfiguration(QGCHilLink* link, QGCHilConfiguration *parent = 0);
    ~QGCHilCondorConfiguration();

public slots:
    /** @brief Start / stop simulation */
    void toggleSimulation(bool connect);
    /** @brief Set X-Plane version */
    void setVersion(int version);

protected:
    QGCHilLink* link;
    
private:
    Ui::QGCHilCondorConfiguration *ui;
};

#endif // QGCHILCONDORCONFIGURATION_H
