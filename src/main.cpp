/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../include/trajectory_planning_gui/main_window.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
    trajectory_planning_gui::MainWindow w(argc,argv);
    w.showMaximized();
    // w.setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
    // hides close/minimize/maximize buttons 
    // w.setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);
    w.show(); 
    // set title
    w.setWindowTitle(QObject::tr("Trajectory Planning GUI"));
    // set app icon
    w.setWindowIcon(QIcon(":images/orangewood.png"));
    // diable maximize button
    //w.setFixedSize(w.width(), w.height());
    //w.setFixedSize(1280, 720);
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    return app.exec();
}
