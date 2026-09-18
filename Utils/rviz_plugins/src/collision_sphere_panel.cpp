#include "collision_sphere_panel.h"
#include <ros/package.h>
#include <pluginlib/class_list_macros.h>
#include <QFormLayout>
#include <QFile>
#include <QTextStream>
#include <QRegularExpression>

namespace { const QStringList kLinks = {"Link1", "Link2", "Link3", "Link4", "Link5", "Link6", "arm_gripper_link"}; }

CollisionSpherePanel::CollisionSpherePanel(QWidget* parent) : rviz::Panel(parent) {
  link_box_ = new QComboBox; link_box_->addItems(kLinks);
  count_box_ = new QSpinBox; count_box_->setRange(1, 32);
  scale_box_ = new QDoubleSpinBox; scale_box_->setRange(0.05, 3.0); scale_box_->setSingleStep(0.05); scale_box_->setDecimals(2);
  auto* layout = new QFormLayout; layout->addRow("Link", link_box_); layout->addRow("Sphere count", count_box_); layout->addRow("Radius scale", scale_box_); setLayout(layout);
  connect(link_box_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CollisionSpherePanel::linkChanged);
  connect(count_box_, QOverload<int>::of(&QSpinBox::valueChanged), this, &CollisionSpherePanel::valuesChanged);
  connect(scale_box_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CollisionSpherePanel::valuesChanged);
  loadConfig();
}

QString CollisionSpherePanel::configPath() const { return QString::fromStdString(ros::package::getPath("mm_config") + "/config/collision_spheres.yaml"); }

void CollisionSpherePanel::loadConfig() {
  QFile file(configPath()); if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
  const QString text = QString::fromUtf8(file.readAll()); const QString link = link_box_->currentText(); updating_ = true;
  const QRegularExpression re("^  " + QRegularExpression::escape(link) + ":\\s*\\n\\s*count:\\s*(\\d+)\\s*\\n\\s*scale:\\s*([0-9.]+)", QRegularExpression::MultilineOption);
  const auto match = re.match(text); if (match.hasMatch()) { count_box_->setValue(match.captured(1).toInt()); scale_box_->setValue(match.captured(2).toDouble()); } updating_ = false;
}

void CollisionSpherePanel::saveConfig() {
  QFile file(configPath()); if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return; QString text = QString::fromUtf8(file.readAll());
  const QString link = link_box_->currentText(); const QRegularExpression re("(^  " + QRegularExpression::escape(link) + ":\\s*\\n\\s*count:)\\s*\\d+(\\s*\\n\\s*scale:)\\s*[0-9.]+", QRegularExpression::MultilineOption);
  text.replace(re, "\\1 " + QString::number(count_box_->value()) + "\\2 " + QString::number(scale_box_->value(), 'f', 2)); file.close();
  if (file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) { QTextStream out(&file); out << text; }
}
void CollisionSpherePanel::linkChanged(int) { loadConfig(); }
void CollisionSpherePanel::valuesChanged() { if (!updating_) saveConfig(); }
PLUGINLIB_EXPORT_CLASS(CollisionSpherePanel, rviz::Panel)
