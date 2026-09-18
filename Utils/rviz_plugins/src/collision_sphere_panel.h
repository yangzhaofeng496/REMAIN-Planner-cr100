#pragma once
#include <rviz/panel.h>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSpinBox>

class CollisionSpherePanel : public rviz::Panel {
  Q_OBJECT
 public:
  explicit CollisionSpherePanel(QWidget* parent = nullptr);
 private Q_SLOTS:
  void linkChanged(int index);
  void valuesChanged();
 private:
  void loadConfig();
  void saveConfig();
  QString configPath() const;
  QComboBox* link_box_;
  QSpinBox* count_box_;
  QDoubleSpinBox* scale_box_;
  bool updating_ = false;
};
