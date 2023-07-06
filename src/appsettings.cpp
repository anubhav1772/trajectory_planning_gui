#include "../include/trajectory_planning_gui/appsettings.h"

#include <string>
#include <QDebug>
#include <QFile>
#include <QTextStream>
#include <fstream>

using json = nlohmann::json;

AppSettings::AppSettings(QObject* parent, const QString& path) : QObject(parent) {
	if (loadFromJSON(path)) isSuccessful = true;
	else isSuccessful = false;
}

AppSettings::~AppSettings() {
}

bool AppSettings::loadFromJSON(const QString& path) {
	std::string rawData;
	std::string readData;
	std::ifstream file;
	file.open(path.toStdString().c_str());
	if (!file) {
		qDebug() << "Couldn't read the file!";
		return false;
	}

	while (file >> readData) {
		rawData += readData;
	}

	auto data = json::parse(rawData);
	if (data.empty()) {
		qDebug() << "Empty json!";
		return false;
	}

	this->path = QString::fromStdString(data["path"].get<std::string>());
	codec = QString::fromStdString(data["codec"].get<std::string>());;
	container = QString::fromStdString(data["container"].get<std::string>());;
	device = data["device"].get<int>();

	return true;
}

void AppSettings::saveToJSON(const QString& path) {
	json data;

	data["path"] = this->path.toStdString();
	data["codec"] = codec.toStdString();
	data["container"] = container.toStdString();
	data["device"] = device;


	std::ofstream file;
	qDebug() << "Path: " << path;
	file.open(path.toStdString());
	file << data;
	file.close();

}
