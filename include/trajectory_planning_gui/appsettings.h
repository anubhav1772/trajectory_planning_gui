#pragma once

#include <QObject>
#include <QUrl>

#include <QString>
#include <QKeySequence>

#include "json.hpp"

class AppSettings : public QObject {
	Q_OBJECT
public:
	AppSettings(QObject* parent, const QString& path);
	~AppSettings();

	bool loadFromJSON(const QString& path);
	void saveToJSON(const QString& path);

	inline QString & getPath() { return path; }
	inline QString & getCodec() { return codec; }
	inline QString & getContainer() { return container; }
	inline int getDevice() { return device; }

	inline void setPath(const QString& path) { this->path = path; }
	inline void setCodec(const QString& codec) { this->codec = codec; }
	inline void setContainer(const QString& container) { this->container = container; }
	inline void setDevice(int device) { this->device = device; }

	bool isSuccessful = false;

private:
	QString path;
	QString codec;
	QString container;
	int device;
};
