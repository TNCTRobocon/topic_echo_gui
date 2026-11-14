#include <QApplication>
#include <QDir>
#include <QFile>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMainWindow>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QPlainTextEdit>
#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>
#include <QScrollBar>
#include <QSignalBlocker>
#include <QStackedWidget>
#include <QPalette>
#include <QColor>
#include <QComboBox>
#include <QDateTime>
#include <QStyleOptionViewItem>
#include <QPushButton>
#include <QSaveFile>
#include <QSplitter>
#include <QStyle>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

using rosidl_typesupport_introspection_cpp::MessageMember;
using rosidl_typesupport_introspection_cpp::MessageMembers;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64;

namespace
{
using NumericFields = std::map<std::string, double>;

struct TopicUpdate
{
  std::string topic;
  std::string text;
  NumericFields numeric_fields;
  int64_t monotonic_timestamp_ns{0};
  int64_t wall_timestamp_ms{0};
};

constexpr size_t kTopicHistoryLimit = 2000;
constexpr size_t kPendingUpdateLimit = 2000;
constexpr size_t kMaxNumericFields = 128;
constexpr size_t kPendingUpdateByteLimit = 4U * 1024U * 1024U;
constexpr size_t kTopicHistoryByteLimit = 16U * 1024U * 1024U;

size_t estimateTopicUpdateBytes(const TopicUpdate & update)
{
  size_t bytes = sizeof(TopicUpdate) + update.text.size();
  for (const auto & [field, value] : update.numeric_fields) {
    Q_UNUSED(value);
    bytes += sizeof(std::pair<const std::string, double>) + 32U + field.size();
  }
  return bytes;
}

std::string spaces(int depth)
{
  return std::string(static_cast<size_t>(depth) * 2U, ' ');
}
std::string scalarToString(uint8_t type, const void * value)
{
  std::ostringstream out;
  out << std::setprecision(std::numeric_limits<double>::max_digits10);
  switch (type) {
    case ROS_TYPE_BOOLEAN: return *static_cast<const bool *>(value) ? "true" : "false";
    case ROS_TYPE_CHAR:
    case ROS_TYPE_OCTET:
    case ROS_TYPE_UINT8:
      out << static_cast<unsigned int>(*static_cast<const uint8_t *>(value)); return out.str();
    case ROS_TYPE_INT8: out << static_cast<int>(*static_cast<const int8_t *>(value)); return out.str();
    case ROS_TYPE_UINT16: out << *static_cast<const uint16_t *>(value); return out.str();
    case ROS_TYPE_INT16: out << *static_cast<const int16_t *>(value); return out.str();
    case ROS_TYPE_UINT32: out << *static_cast<const uint32_t *>(value); return out.str();
    case ROS_TYPE_INT32: out << *static_cast<const int32_t *>(value); return out.str();
    case ROS_TYPE_UINT64: out << *static_cast<const uint64_t *>(value); return out.str();
    case ROS_TYPE_INT64: out << *static_cast<const int64_t *>(value); return out.str();
    case ROS_TYPE_FLOAT: out << *static_cast<const float *>(value); return out.str();
    case ROS_TYPE_DOUBLE: out << *static_cast<const double *>(value); return out.str();
    case ROS_TYPE_STRING: return *static_cast<const std::string *>(value);
    default: return "<unsupported>";
  }
}

std::string renderMessage(const MessageMembers * members, const void * message, int depth);

std::string renderField(const MessageMember & member, const void * field, int depth)
{
  std::ostringstream out;
  if (member.is_array_) {
    const size_t count = member.size_function(field);
    out << "\n";
    for (size_t i = 0; i < count; ++i) {
      const void * item = member.get_const_function(field, i);
      out << spaces(depth + 1) << "-";
      if (member.type_id_ == ROS_TYPE_MESSAGE) {
        out << renderMessage(
          static_cast<const MessageMembers *>(member.members_->data), item, depth + 2);
      } else {
        out << " " << scalarToString(member.type_id_, item) << "\n";
      }
    }
    return out.str();
  }

  if (member.type_id_ == ROS_TYPE_MESSAGE) {
    return "\n" + renderMessage(
      static_cast<const MessageMembers *>(member.members_->data), field, depth + 1);
  }
  return " " + scalarToString(member.type_id_, field) + "\n";
}
std::string renderMessage(const MessageMembers * members, const void * message, int depth)
{
  std::ostringstream out;
  for (uint32_t i = 0; i < members->member_count_; ++i) {
    const auto & member = members->members_[i];
    const void * field =
      reinterpret_cast<const unsigned char *>(message) + member.offset_;
    out << spaces(depth) << member.name_ << ":";
    out << renderField(member, field, depth);
  }
  return out.str();
}

bool numericFieldToDouble(uint8_t type, const void * value, double & result)
{
  switch (type) {
    case ROS_TYPE_CHAR:
    case ROS_TYPE_OCTET:
    case ROS_TYPE_UINT8:
      result = static_cast<double>(*static_cast<const uint8_t *>(value)); return true;
    case ROS_TYPE_INT8:
      result = static_cast<double>(*static_cast<const int8_t *>(value)); return true;
    case ROS_TYPE_UINT16:
      result = static_cast<double>(*static_cast<const uint16_t *>(value)); return true;
    case ROS_TYPE_INT16:
      result = static_cast<double>(*static_cast<const int16_t *>(value)); return true;
    case ROS_TYPE_UINT32:
      result = static_cast<double>(*static_cast<const uint32_t *>(value)); return true;
    case ROS_TYPE_INT32:
      result = static_cast<double>(*static_cast<const int32_t *>(value)); return true;
    case ROS_TYPE_UINT64:
      result = static_cast<double>(*static_cast<const uint64_t *>(value)); return true;
    case ROS_TYPE_INT64:
      result = static_cast<double>(*static_cast<const int64_t *>(value)); return true;
    case ROS_TYPE_FLOAT:
      result = static_cast<double>(*static_cast<const float *>(value)); return std::isfinite(result);
    case ROS_TYPE_DOUBLE:
      result = *static_cast<const double *>(value); return std::isfinite(result);
    default:
      return false;
  }
}

void collectNumericFields(
  const MessageMembers * members, const void * message, const std::string & prefix,
  NumericFields & result, size_t max_fields = kMaxNumericFields)
{
  for (uint32_t i = 0; i < members->member_count_ && result.size() < max_fields; ++i) {
    const auto & member = members->members_[i];
    const void * field =
      reinterpret_cast<const unsigned char *>(message) + member.offset_;
    const std::string name = prefix.empty() ? member.name_ : prefix + "." + member.name_;

    if (member.is_array_) {
      const size_t count = member.size_function(field);
      for (size_t index = 0; index < count && result.size() < max_fields; ++index) {
        const void * item = member.get_const_function(field, index);
        const std::string item_name = name + "[" + std::to_string(index) + "]";
        if (member.type_id_ == ROS_TYPE_MESSAGE) {
          collectNumericFields(
            static_cast<const MessageMembers *>(member.members_->data), item, item_name,
            result, max_fields);
        } else {
          double value = 0.0;
          if (numericFieldToDouble(member.type_id_, item, value)) {
            result[item_name] = value;
          }
        }
      }
    } else if (member.type_id_ == ROS_TYPE_MESSAGE) {
      collectNumericFields(
        static_cast<const MessageMembers *>(member.members_->data), field, name,
        result, max_fields);
    } else {
      double value = 0.0;
      if (numericFieldToDouble(member.type_id_, field, value)) {
        result[name] = value;
      }
    }
  }
}

NumericFields collectJoyNumericFields(const sensor_msgs::msg::Joy & msg)
{
  NumericFields result;
  for (size_t i = 0; i < msg.axes.size(); ++i) {
    if (std::isfinite(msg.axes[i])) {
      result["axes[" + std::to_string(i) + "]"] = msg.axes[i];
    }
  }
  for (size_t i = 0; i < msg.buttons.size(); ++i) {
    result["buttons[" + std::to_string(i) + "]"] =
      static_cast<double>(msg.buttons[i]);
  }
  return result;
}

std::string formatJoy(const sensor_msgs::msg::Joy & msg)
{
  std::ostringstream out;
  out << "header:\n";
  out << "  stamp:\n";
  out << "    sec: " << msg.header.stamp.sec << "\n";
  out << "    nanosec: " << msg.header.stamp.nanosec << "\n";
  out << "  frame_id: " << msg.header.frame_id << "\n";
  out << "axes:\n";
  out << std::setprecision(std::numeric_limits<float>::max_digits10);
  for (float v : msg.axes) out << "- " << v << "\n";
  out << "buttons:\n";
  for (int32_t v : msg.buttons) out << "- " << v << "\n";
  return out.str();
}

class TopicListWidget final : public QListWidget
{
public:
  using QListWidget::QListWidget;
  std::function<void(QListWidgetItem *)> onTopicClicked;

protected:
  bool isCheckIndicatorAt(const QPoint & position) const
  {
    QListWidgetItem * item = itemAt(position);
    if (item == nullptr) return false;

    QStyleOptionViewItem option = viewOptions();
    option.rect = visualItemRect(item);
    option.features |= QStyleOptionViewItem::HasCheckIndicator;
    option.checkState = item->checkState();
    const QRect checkRect = style()->subElementRect(
      QStyle::SE_ItemViewItemCheckIndicator, &option, viewport());
    return checkRect.contains(position);
  }

  void mousePressEvent(QMouseEvent * event) override
  {
    swallow_check_click_ =
      event->button() == Qt::LeftButton && isCheckIndicatorAt(event->pos());
    if (swallow_check_click_) {
      event->accept();
      return;
    }
    QListWidget::mousePressEvent(event);
  }

  void mouseReleaseEvent(QMouseEvent * event) override
  {
    const bool checkPress = swallow_check_click_;
    const bool checkRelease =
      event->button() == Qt::LeftButton && isCheckIndicatorAt(event->pos());
    if (checkPress || checkRelease) {
      QListWidgetItem * clickedItem =
        checkPress && checkRelease ? itemAt(event->pos()) : nullptr;
      swallow_check_click_ = false;
      event->accept();
      // Keep Qt from toggling the check state directly. The shared topic
      // assignment callback updates the indicator from the actual assignment.
      if (clickedItem != nullptr && onTopicClicked) {
        onTopicClicked(clickedItem);
      }
      return;
    }

    QListWidgetItem * clickedItem = itemAt(event->pos());
    const bool itemClick =
      event->button() == Qt::LeftButton && clickedItem != nullptr;
    QListWidget::mouseReleaseEvent(event);
    if (itemClick && onTopicClicked) {
      onTopicClicked(clickedItem);
    }
  }

  void keyPressEvent(QKeyEvent * event) override
  {
    // The checkbox only indicates actual subscription state; it is not an input.
    if (event->key() == Qt::Key_Space) {
      event->accept();
      return;
    }
    QListWidget::keyPressEvent(event);
  }

private:
  bool swallow_check_click_{false};
};

class EchoNode final : public rclcpp::Node
{
public:
  EchoNode()
  : Node("topic_echo_gui"),
    qos_(rclcpp::QoS(10).best_effort().durability_volatile())
  {}

  std::vector<std::pair<std::string, std::vector<std::string>>> topics() const
  {
    std::vector<std::pair<std::string, std::vector<std::string>>> result;
    for (const auto & item : get_topic_names_and_types()) {
      if (item.first == "/rosout" || item.first == "/parameter_events") continue;
      result.push_back(item);
    }
    std::sort(result.begin(), result.end(),
      [](const auto & a, const auto & b) { return a.first < b.first; });
    return result;
  }

  std::vector<std::string> startEcho(
    const std::vector<std::pair<std::string, std::string>> & selections)
  {
    stopEcho();
    std::vector<std::string> errors;
    for (const auto & [name, type] : selections) {
      const std::string error = addSubscription(name, type);
      if (!error.empty()) errors.push_back(error);
    }
    return errors;
  }

  std::string addSubscription(const std::string & name, const std::string & type)
  {
    if (subscriptions_.count(name) > 0) return {};
    try {
      if (type == "sensor_msgs/msg/Joy") {
        auto sub = create_subscription<sensor_msgs::msg::Joy>(
          name, qos_,
          [this, name](sensor_msgs::msg::Joy::ConstSharedPtr msg) {
            enqueueUpdate(name, formatJoy(*msg), collectJoyNumericFields(*msg));
          });
        subscriptions_[name] = sub;
      } else {
        auto sub = create_generic_subscription(
          name, type, qos_,
          [this, name, type](std::shared_ptr<rclcpp::SerializedMessage> msg) {
            renderGeneric(name, type, msg);
          });
        subscriptions_[name] = sub;
      }
    } catch (const std::exception & e) {
      return name + " [" + type + "]: " + e.what();
    }
    return {};
  }

  void removeSubscription(const std::string & name)
  {
    subscriptions_.erase(name);
    std::lock_guard<std::mutex> lock(updates_mutex_);
    pending_updates_.erase(name);
    pending_update_bytes_.erase(name);
  }

  bool hasSubscription(const std::string & name) const
  {
    return subscriptions_.count(name) > 0;
  }

  std::set<std::string> subscriptionNames() const
  {
    std::set<std::string> names;
    for (const auto & [name, subscription] : subscriptions_) {
      Q_UNUSED(subscription);
      names.insert(name);
    }
    return names;
  }

  void stopEcho()
  {
    subscriptions_.clear();
    std::lock_guard<std::mutex> lock(updates_mutex_);
    pending_updates_.clear();
    pending_update_bytes_.clear();
  }

  std::vector<TopicUpdate> takePendingUpdates()
  {
    std::vector<TopicUpdate> updates;
    std::lock_guard<std::mutex> lock(updates_mutex_);
    for (auto & [topic, queue] : pending_updates_) {
      Q_UNUSED(topic);
      for (auto & update : queue) updates.push_back(std::move(update));
    }
    pending_updates_.clear();
    pending_update_bytes_.clear();
    return updates;
  }

  void clearPendingUpdates()
  {
    std::lock_guard<std::mutex> lock(updates_mutex_);
    pending_updates_.clear();
    pending_update_bytes_.clear();
  }

private:
  void enqueueUpdate(
    const std::string & topic, const std::string & text,
    NumericFields numeric_fields)
  {
    const auto monotonic_now = std::chrono::steady_clock::now().time_since_epoch();
    const auto wall_now = std::chrono::system_clock::now().time_since_epoch();
    TopicUpdate update;
    update.topic = topic;
    update.text = text;
    update.numeric_fields = std::move(numeric_fields);
    update.monotonic_timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      monotonic_now).count();
    update.wall_timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      wall_now).count();

    const size_t update_bytes = estimateTopicUpdateBytes(update);
    std::lock_guard<std::mutex> lock(updates_mutex_);
    auto & pending = pending_updates_[topic];
    auto & pending_bytes = pending_update_bytes_[topic];
    pending.push_back(std::move(update));
    pending_bytes += update_bytes;
    while (pending.size() > 1 &&
      (pending.size() > kPendingUpdateLimit ||
      pending_bytes > kPendingUpdateByteLimit))
    {
      pending_bytes -= estimateTopicUpdateBytes(pending.front());
      pending.pop_front();
    }
  }

  void renderGeneric(
    const std::string & topic,
    const std::string & type,
    const std::shared_ptr<rclcpp::SerializedMessage> & serialized)
  {
    try {
      auto cpp_lib = rclcpp::get_typesupport_library(type, "rosidl_typesupport_cpp");
      auto cpp_ts = rclcpp::get_message_typesupport_handle(
        type, "rosidl_typesupport_cpp", *cpp_lib);
      auto intro_lib =
        rclcpp::get_typesupport_library(type, "rosidl_typesupport_introspection_cpp");
      auto intro_ts = rclcpp::get_message_typesupport_handle(
        type, "rosidl_typesupport_introspection_cpp", *intro_lib);
      const auto * members = static_cast<const MessageMembers *>(intro_ts->data);

      const size_t alignment = alignof(std::max_align_t);
      const size_t size = std::max(members->size_of_, sizeof(void *));
      const size_t alloc_size = ((size + alignment - 1U) / alignment) * alignment;
      void * memory = std::aligned_alloc(alignment, alloc_size);
      if (memory == nullptr) throw std::bad_alloc();

      members->init_function(memory, rosidl_runtime_cpp::MessageInitialization::ALL);
      try {
        rclcpp::SerializationBase serializer(cpp_ts);
        serializer.deserialize_message(serialized.get(), memory);
        NumericFields numeric_fields;
        collectNumericFields(members, memory, "", numeric_fields);
        enqueueUpdate(topic, renderMessage(members, memory, 0), std::move(numeric_fields));
      } catch (...) {
        members->fini_function(memory);
        std::free(memory);
        throw;
      }
      members->fini_function(memory);
      std::free(memory);
    } catch (const std::exception & e) {
      enqueueUpdate(topic, std::string("表示変換エラー: ") + e.what(), {});
    }
  }

  rclcpp::QoS qos_;
  std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  mutable std::mutex updates_mutex_;
  std::map<std::string, std::deque<TopicUpdate>> pending_updates_;
  std::map<std::string, size_t> pending_update_bytes_;
};

class NumericGraphWidget final : public QWidget
{
  struct Point
  {
    int64_t monotonic_ns{0};
    int64_t wall_time_ms{0};
    double value{0.0};
  };

public:
  explicit NumericGraphWidget(QWidget * parent = nullptr)
  : QWidget(parent)
  {
    setMinimumSize(80, 60);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  }

  void setSelectedField(const std::string & field)
  {
    if (selected_field_ == field) return;
    selected_field_ = field;
    update();
  }

  void addSample(const TopicUpdate & sample)
  {
    for (const auto & [field, value] : sample.numeric_fields) {
      auto & points = series_[field];
      points.push_back(Point{
        sample.monotonic_timestamp_ns, sample.wall_timestamp_ms, value});
      while (points.size() > kTopicHistoryLimit) points.pop_front();
    }
    has_message_ = true;
    status_message_.clear();
    update();
  }

  void setStatusMessage(const QString & message)
  {
    if (status_message_ == message) return;
    status_message_ = message;
    update();
  }

  void clearHistory()
  {
    series_.clear();
    has_message_ = false;
    update();
  }

protected:
  void paintEvent(QPaintEvent * event) override
  {
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), QColor("#14191f"));

    const auto series_it = series_.find(selected_field_);
    if (!status_message_.isEmpty()) {
      drawCenteredMessage(painter, status_message_);
      return;
    }
    if (selected_field_.empty() || series_.empty()) {
      drawCenteredMessage(
        painter, has_message_ ? "グラフ化できる数値フィールドがありません" :
        "数値データ受信待ち...");
      return;
    }
    if (series_it == series_.end() || series_it->second.empty()) {
      drawCenteredMessage(painter, "選択されたフィールドの数値データがありません");
      return;
    }

    const auto & points = series_it->second;
    const int left = 64;
    const int right = 14;
    const int top = 14;
    const int bottom = 34;
    const QRectF plot(
      left, top, std::max(1, width() - left - right),
      std::max(1, height() - top - bottom));

    double min_value = points.front().value;
    double max_value = points.front().value;
    for (const auto & point : points) {
      min_value = std::min(min_value, point.value);
      max_value = std::max(max_value, point.value);
    }
    double range = max_value - min_value;
    const double padding = range > 0.0 ? range * 0.08 :
      std::max(1.0, std::abs(max_value) * 0.08);
    min_value -= padding;
    max_value += padding;
    range = max_value - min_value;

    painter.setFont(QFont("sans-serif", 8));
    QPen grid_pen(QColor("#303841"));
    grid_pen.setWidth(1);
    painter.setPen(grid_pen);
    for (int tick = 0; tick <= 4; ++tick) {
      const qreal y = plot.top() + plot.height() * tick / 4.0;
      painter.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y));
      const double value = max_value - range * tick / 4.0;
      painter.setPen(QColor("#aeb8c4"));
      painter.drawText(
        QRectF(2, y - 9, left - 10, 18), Qt::AlignRight | Qt::AlignVCenter,
        QString::number(value, 'g', 4));
      painter.setPen(grid_pen);
    }
    painter.drawRect(plot);

    const int64_t first_ns = points.front().monotonic_ns;
    const int64_t last_ns = points.back().monotonic_ns;
    const int64_t time_span = std::max<int64_t>(1, last_ns - first_ns);
    QPainterPath path;
    for (size_t i = 0; i < points.size(); ++i) {
      const auto & point = points[i];
      const qreal x = plot.left() + plot.width() *
        static_cast<qreal>(point.monotonic_ns - first_ns) / time_span;
      const qreal y = plot.bottom() - plot.height() *
        static_cast<qreal>(point.value - min_value) / range;
      if (i == 0) path.moveTo(x, y);
      else path.lineTo(x, y);
    }
    QPen line_pen(QColor("#58a6ff"));
    line_pen.setWidth(2);
    painter.setPen(line_pen);
    painter.drawPath(path);
    if (points.size() == 1) {
      const auto & point = points.front();
      const qreal x = plot.left();
      const qreal y = plot.bottom() - plot.height() *
        static_cast<qreal>(point.value - min_value) / range;
      painter.setBrush(QColor("#58a6ff"));
      painter.drawEllipse(QPointF(x, y), 3.5, 3.5);
    }

    painter.setPen(QColor("#aeb8c4"));
    const QString first_time = QDateTime::fromMSecsSinceEpoch(
      points.front().wall_time_ms).toString("HH:mm:ss");
    const QString last_time = QDateTime::fromMSecsSinceEpoch(
      points.back().wall_time_ms).toString("HH:mm:ss");
    painter.drawText(
      QRectF(plot.left(), plot.bottom() + 5, plot.width() / 2.0, 20),
      Qt::AlignLeft | Qt::AlignTop, first_time);
    painter.drawText(
      QRectF(plot.left() + plot.width() / 2.0, plot.bottom() + 5,
        plot.width() / 2.0, 20), Qt::AlignRight | Qt::AlignTop, last_time);
  }

private:
  void drawCenteredMessage(QPainter & painter, const QString & message)
  {
    painter.setPen(QColor("#89939e"));
    painter.drawText(rect().adjusted(8, 8, -8, -8), Qt::AlignCenter, message);
  }

  std::string selected_field_;
  std::map<std::string, std::deque<Point>> series_;
  QString status_message_;
  bool has_message_{false};
};

class MainWindow final : public QMainWindow
{
  struct TopicDisplayWidgets
  {
    QWidget * card{nullptr};
    QComboBox * field_selector{nullptr};
    QStackedWidget * stack{nullptr};
    QPlainTextEdit * text_view{nullptr};
    NumericGraphWidget * graph_view{nullptr};
    std::vector<std::string> available_fields;
    bool has_received{false};
    QString placeholder;
  };

  struct LayoutNode
  {
    int id{0};
    bool is_split{false};
    Qt::Orientation orientation{Qt::Horizontal};
    int ratio{500};
    std::vector<std::string> topics;
    std::unique_ptr<LayoutNode> first;
    std::unique_ptr<LayoutNode> second;
  };

public:
  explicit MainWindow(std::shared_ptr<EchoNode> node)
  : node_(std::move(node)),
    layout_root_(std::make_unique<LayoutNode>()),
    next_layout_id_(2)
  {
    layout_root_->id = 1;
    setWindowTitle("ROS 2 Topic Echo GUI");
    resize(1100, 700);
    setMinimumSize(850, 520);

    // Apply a consistent dark palette to the complete application window.
    QPalette darkPalette;
    darkPalette.setColor(QPalette::Window, QColor("#1b1f24"));
    darkPalette.setColor(QPalette::WindowText, QColor("#e6e9ed"));
    darkPalette.setColor(QPalette::Base, QColor("#14191f"));
    darkPalette.setColor(QPalette::AlternateBase, QColor("#20262e"));
    darkPalette.setColor(QPalette::ToolTipBase, QColor("#2b3138"));
    darkPalette.setColor(QPalette::ToolTipText, QColor("#f0f2f5"));
    darkPalette.setColor(QPalette::Text, QColor("#e6e9ed"));
    darkPalette.setColor(QPalette::Button, QColor("#2b3138"));
    darkPalette.setColor(QPalette::ButtonText, QColor("#e6e9ed"));
    darkPalette.setColor(QPalette::BrightText, QColor("#ff6b6b"));
    darkPalette.setColor(QPalette::Link, QColor("#76b7ff"));
    darkPalette.setColor(QPalette::Highlight, QColor("#2f6fad"));
    darkPalette.setColor(QPalette::HighlightedText, QColor("#ffffff"));
    darkPalette.setColor(QPalette::PlaceholderText, QColor("#89939e"));
    darkPalette.setColor(QPalette::Light, QColor("#424b56"));
    darkPalette.setColor(QPalette::Midlight, QColor("#343c46"));
    darkPalette.setColor(QPalette::Mid, QColor("#303841"));
    darkPalette.setColor(QPalette::Dark, QColor("#171b20"));
    darkPalette.setColor(QPalette::Shadow, QColor("#0b0e12"));
    darkPalette.setColor(QPalette::Disabled, QPalette::WindowText, QColor("#77818c"));
    darkPalette.setColor(QPalette::Disabled, QPalette::Text, QColor("#77818c"));
    darkPalette.setColor(QPalette::Disabled, QPalette::ButtonText, QColor("#77818c"));
    QApplication::setPalette(darkPalette);
    setPalette(darkPalette);
    setStyleSheet(R"(
      QMainWindow, QWidget {
        background-color: #1b1f24;
        color: #e6e9ed;
      }
      QLabel {
        background-color: transparent;
        color: #e6e9ed;
      }
      QPushButton {
        background-color: #2b3138;
        color: #e6e9ed;
        border: 1px solid #596574;
        border-radius: 6px;
        padding: 6px 11px;
      }
      QPushButton:hover {
        background-color: #39434f;
        border-color: #4b91cc;
      }
      QPushButton:pressed {
        background-color: #234d70;
        border-color: #2f6fad;
      }
      QPushButton:checked {
        background-color: #2f6fad;
        color: #ffffff;
        border-color: #4b91cc;
      }
      QPushButton:disabled {
        background-color: #252a30;
        color: #77818c;
        border-color: #3a424c;
      }
      QLineEdit, QListWidget, QPlainTextEdit {
        background-color: #14191f;
        color: #e6e9ed;
        border: 1px solid #46515e;
        border-radius: 4px;
        selection-background-color: #2f6fad;
        selection-color: #ffffff;
        padding: 4px;
      }
      QLineEdit:focus, QPlainTextEdit:focus, QComboBox:focus {
        border-color: #4b91cc;
      }
      QComboBox {
        background-color: #14191f;
        color: #e6e9ed;
        border: 1px solid #46515e;
        border-radius: 4px;
        padding: 3px 6px;
      }
      QComboBox::drop-down {
        border: none;
        width: 22px;
      }
      QComboBox QAbstractItemView {
        background-color: #20262e;
        color: #e6e9ed;
        selection-background-color: #2f6fad;
        selection-color: #ffffff;
        border: 1px solid #46515e;
      }
      QWidget#topicDisplayCard {
        background-color: #1b1f24;
        border: 1px solid #46515e;
        border-radius: 5px;
      }
      QListWidget::item {
        padding: 5px 4px;
        border-bottom: 1px solid #29313a;
      }
      QListWidget::item:hover {
        background-color: #28313b;
      }
      QScrollBar:vertical {
        background-color: #171c22;
        width: 12px;
        margin: 0;
      }
      QScrollBar:horizontal {
        background-color: #171c22;
        height: 12px;
        margin: 0;
      }
      QScrollBar::handle:vertical, QScrollBar::handle:horizontal {
        background-color: #485462;
        border-radius: 5px;
        min-width: 24px;
        min-height: 24px;
      }
      QScrollBar::handle:vertical:hover, QScrollBar::handle:horizontal:hover {
        background-color: #627386;
      }
      QScrollBar::add-line, QScrollBar::sub-line {
        background: none;
        border: none;
      }
      QScrollBar::add-page, QScrollBar::sub-page {
        background: none;
      }
      QSplitter::handle {
        background-color: #3a424c;
      }
      QSplitter::handle:hover {
        background-color: #4b91cc;
      }
      QToolTip {
        background-color: #2b3138;
        color: #f0f2f5;
        border: 1px solid #596574;
        padding: 4px;
      }
    )");

    auto * central = new QWidget(this);
    auto * root = new QVBoxLayout(central);
    auto * toolbar = new QHBoxLayout();
    auto * content = new QHBoxLayout();
    auto * layoutToolbar = new QHBoxLayout();

    auto * refresh = new QPushButton("更新");
    auto * clearSel = new QPushButton("すべて選択解除");
    auto * toggleTopics = new QPushButton("Topic一覧を隠す");
    auto * start = new QPushButton("echo開始");
    auto * stop = new QPushButton("echo停止");
    auto * clearOutput = new QPushButton("表示クリア");
    display_mode_button_ = new QPushButton("表示方式：テキスト");
    auto * splitHorizontal = new QPushButton("左右に分割");
    auto * splitVertical = new QPushButton("上下に分割");
    auto * deletePane = new QPushButton("削除");
    search_ = new QLineEdit();
    search_->setPlaceholderText("Topic名または型を検索");

    toolbar->addWidget(refresh);
    toolbar->addWidget(clearSel);
    toolbar->addWidget(toggleTopics);
    toolbar->addWidget(start);
    toolbar->addWidget(stop);
    toolbar->addWidget(clearOutput);
    toolbar->addWidget(display_mode_button_);
    toolbar->addSpacing(12);
    toolbar->addWidget(new QLabel("検索:"));
    toolbar->addWidget(search_, 1);

    layoutToolbar->addWidget(new QLabel("表示枠操作:"));
    layoutToolbar->addWidget(splitHorizontal);
    layoutToolbar->addWidget(splitVertical);
    layoutToolbar->addWidget(deletePane);
    layoutToolbar->addStretch(1);

    topics_ = new TopicListWidget();
    topics_->setSelectionMode(QAbstractItemView::NoSelection);
    topics_->onTopicClicked = [this](QListWidgetItem * item) {
      assignTopicToSelectedPane(item);
    };
    output_container_ = new QWidget();
    output_layout_ = new QVBoxLayout(output_container_);
    output_layout_->setContentsMargins(0, 0, 0, 0);
    output_layout_->setSpacing(0);
    status_ = new QLabel("起動中...");
    topics_panel_ = new QWidget();
    auto * left = new QVBoxLayout(topics_panel_);
    left->setContentsMargins(0, 0, 0, 0);
    left->addWidget(new QLabel("Topic一覧"));
    left->addWidget(topics_);
    auto * right = new QVBoxLayout();
    right->addWidget(new QLabel("Echo出力"));
    right->addWidget(output_container_, 1);
    content->addWidget(topics_panel_, 2);
    content->addLayout(right, 5);

    root->addLayout(toolbar);
    root->addLayout(layoutToolbar);
    root->addLayout(content, 1);
    root->addWidget(status_);
    setCentralWidget(central);

    connect(refresh, &QPushButton::clicked, this, [this]() { refreshTopics(); });
    connect(clearSel, &QPushButton::clicked, this, [this]() {
      clearTopicAssignments(layout_root_.get());
      rebuildLayoutWidget();
      QString errors;
      if (echo_active_) {
        errors = syncSubscriptionsWithAssignments();
      } else {
        syncTopicCheckStates();
        refreshOutput();
      }
      saveTopicSelection();
      if (!errors.isEmpty()) {
        status_->setText("割当解除後、一部の購読処理でエラーが発生しました:\n" + errors);
      } else {
        status_->setText("すべての表示枠からトピックの割当を解除しました。");
      }
    });
    connect(toggleTopics, &QPushButton::clicked, this, [this, toggleTopics]() {
      const bool hidden = topics_panel_->isVisible();
      topics_panel_->setVisible(!hidden);
      toggleTopics->setText(hidden ? "Topic一覧を表示" : "Topic一覧を隠す");
    });
    connect(start, &QPushButton::clicked, this, [this]() { startEcho(); });
    connect(display_mode_button_, &QPushButton::clicked, this, [this]() {
      setGraphMode(!graph_mode_);
    });
    connect(splitHorizontal, &QPushButton::clicked, this, [this]() {
      splitSelectedPane(Qt::Horizontal);
    });
    connect(splitVertical, &QPushButton::clicked, this, [this]() {
      splitSelectedPane(Qt::Vertical);
    });
    connect(deletePane, &QPushButton::clicked, this, [this]() {
      deleteSelectedPane();
    });
    connect(stop, &QPushButton::clicked, this, [this]() {
      node_->stopEcho();
      echo_active_ = false;
      syncTopicCheckStates();
      refreshOutput();
      status_->setText("Echo停止中（受信履歴は保持）");
    });
    connect(clearOutput, &QPushButton::clicked, this, [this]() {
      node_->clearPendingUpdates();
      for (auto & [topic, history] : topic_histories_) {
        history.clear();
        topic_history_bytes_[topic] = 0;
      }
      for (auto & [topic, view] : topic_display_widgets_) {
        view.text_view->clear();
        view.graph_view->clearHistory();
        view.has_received = false;
        view.placeholder.clear();
        topic_scroll_states_[topic] = {0, true};
      }
      refreshOutput();
    });
    connect(search_, &QLineEdit::textChanged, this, [this](const QString &) {
      applyFilter();
    });

    timer_ = new QTimer(this);
    timer_->setTimerType(Qt::PreciseTimer);
    connect(timer_, &QTimer::timeout, this, [this]() { refreshOutput(); });
    timer_->start(2);

    layout_save_timer_ = new QTimer(this);
    layout_save_timer_->setSingleShot(true);
    layout_save_timer_->setInterval(400);
    connect(layout_save_timer_, &QTimer::timeout, this, [this]() {
      saveTopicSelection();
    });

    loadTopicSelection();
    rebuildLayoutWidget();

    // Let ROS 2 discovery populate the graph before the first topic refresh.
    // Trigger the actual button so startup and manual refresh use the same path.
    QTimer::singleShot(1000, this, [refresh]() {
      refresh->click();
    });
    if (std::getenv("TOPIC_ECHO_GUI_AUTO_JOY") != nullptr) {
      QTimer::singleShot(3000, this, [this]() {
        refreshTopics();
        for (int i = 0; i < topics_->count(); ++i) {
          auto * item = topics_->item(i);
          if (item->data(Qt::UserRole).toString() == "/joy") {
            LayoutNode * display = findLayoutNode(
              layout_root_.get(), selected_layout_id_);
            if (display == nullptr || display->is_split) {
              selected_layout_id_ = firstLeafId(layout_root_.get());
              display = findLayoutNode(layout_root_.get(), selected_layout_id_);
            }
            if (display != nullptr && !display->is_split) {
              removeTopicFromAssignments(layout_root_.get(), "/joy");
              display = findLayoutNode(layout_root_.get(), selected_layout_id_);
              display->topics.push_back("/joy");
              rebuildLayoutWidget();
              saveTopicSelection();
            }
            startEcho();
            break;
          }
        }
      });
    }
  }

  ~MainWindow() override
  {
    saveTopicSelection();
    node_->stopEcho();
  }

protected:
  bool eventFilter(QObject * watched, QEvent * event) override
  {
    if (event != nullptr && event->type() == QEvent::MouseButtonPress) {
      const auto * mouseEvent = static_cast<QMouseEvent *>(event);
      if (mouseEvent->button() == Qt::LeftButton) {
        QWidget * widget = qobject_cast<QWidget *>(watched);
        while (widget != nullptr && widget->objectName() != "topicPane") {
          widget = widget->parentWidget();
        }
        if (widget != nullptr) {
          for (const auto & [id, pane] : pane_widgets_) {
            if (pane == widget) {
              if (id != selected_layout_id_) selectLayoutPane(id);
              break;
            }
          }
        }
      }
    }
    return QMainWindow::eventFilter(watched, event);
  }

private:
  QString topicSelectionFilePath() const
  {
    const QString configDirectory =
      QDir::homePath() + "/.config/topic_echo_gui";
    return QDir(configDirectory).filePath("settings.json");
  }

  void loadTopicSelection()
  {
    QFile file(topicSelectionFilePath());
    if (!file.exists()) return;
    if (!file.open(QIODevice::ReadOnly)) {
      RCLCPP_WARN(
        node_->get_logger(), "Topic selection settings could not be opened: %s",
        file.fileName().toStdString().c_str());
      return;
    }

    QJsonParseError parseError;
    const QJsonDocument document =
      QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject()) {
      RCLCPP_WARN(
        node_->get_logger(), "Topic selection settings are invalid JSON: %s",
        parseError.errorString().toStdString().c_str());
      return;
    }

    const QJsonObject settings = document.object();

    next_layout_id_ = 1;
    auto restoredLayout = layoutFromJson(settings.value("layout"));
    if (restoredLayout) {
      layout_root_ = std::move(restoredLayout);
    } else {
      layout_root_ = std::make_unique<LayoutNode>();
      layout_root_->id = next_layout_id_++;
    }

    std::set<std::string> seenTopics;
    normalizeTopicAssignments(layout_root_.get(), seenTopics);
    selected_layout_id_ = settings.value("selected_pane_id").toInt(layout_root_->id);
  }

  std::unique_ptr<LayoutNode> layoutFromJson(const QJsonValue & value)
  {
    if (!value.isObject()) return nullptr;
    const QJsonObject object = value.toObject();
    auto node = std::make_unique<LayoutNode>();

    int id = object.value("id").toInt(0);
    if (id <= 0) id = next_layout_id_++;
    node->id = id;
    next_layout_id_ = std::max(next_layout_id_, id + 1);

    const QString kind = object.value("kind").toString();
    if (kind == "split") {
      auto first = layoutFromJson(object.value("first"));
      auto second = layoutFromJson(object.value("second"));
      if (first && second) {
        node->is_split = true;
        node->orientation =
          object.value("orientation").toString() == "vertical" ?
          Qt::Vertical : Qt::Horizontal;
        node->ratio = std::clamp(object.value("ratio").toInt(500), 100, 900);
        node->first = std::move(first);
        node->second = std::move(second);
        return node;
      }
    }

    node->is_split = false;
    const QJsonValue topicsValue = object.value("topics");
    if (topicsValue.isArray()) {
      for (const QJsonValue & topicValue : topicsValue.toArray()) {
        if (!topicValue.isString()) continue;
        const std::string topic = topicValue.toString().toStdString();
        if (!topic.empty()) node->topics.push_back(topic);
      }
    } else {
      // Compatibility with settings saved by older one-topic-per-pane versions.
      const std::string oldTopic = object.value("topic").toString().toStdString();
      if (!oldTopic.empty()) node->topics.push_back(oldTopic);
    }
    return node;
  }

  QJsonObject layoutToJson(const LayoutNode & node) const
  {
    QJsonObject object;
    object.insert("id", node.id);
    if (node.is_split && node.first && node.second) {
      object.insert("kind", "split");
      object.insert(
        "orientation",
        node.orientation == Qt::Vertical ? "vertical" : "horizontal");
      object.insert("ratio", node.ratio);
      object.insert("first", layoutToJson(*node.first));
      object.insert("second", layoutToJson(*node.second));
    } else {
      object.insert("kind", "leaf");
      QJsonArray topics;
      for (const auto & topic : node.topics) {
        topics.append(QString::fromStdString(topic));
      }
      object.insert("topics", topics);
    }
    return object;
  }

  void saveTopicSelection()
  {
    captureSplitterRatios();
    const QString configDirectory =
      QDir::homePath() + "/.config/topic_echo_gui";
    if (!QDir().mkpath(configDirectory)) {
      RCLCPP_WARN(
        node_->get_logger(), "Could not create topic selection settings directory: %s",
        configDirectory.toStdString().c_str());
      return;
    }

    QJsonObject settings;
    if (layout_root_) {
      settings.insert("layout", layoutToJson(*layout_root_));
      settings.insert("selected_pane_id", selected_layout_id_);
    }
    const QByteArray json = QJsonDocument(settings).toJson(QJsonDocument::Indented);

    QSaveFile file(topicSelectionFilePath());
    if (!file.open(QIODevice::WriteOnly)) {
      RCLCPP_WARN(
        node_->get_logger(), "Topic selection settings could not be saved: %s",
        file.fileName().toStdString().c_str());
      return;
    }
    if (file.write(json) != json.size() || !file.commit()) {
      RCLCPP_WARN(
        node_->get_logger(), "Failed to write topic selection settings: %s",
        file.fileName().toStdString().c_str());
    }
  }

  void refreshTopics()
  {
    const bool wasLoadingTopics = loading_topics_;
    loading_topics_ = true;

    topics_->clear();
    topic_types_.clear();
    for (const auto & [name, types] : node_->topics()) {
      const std::string type = types.empty() ? "" : types.front();
      topic_types_[name] = type;
      auto * item = new QListWidgetItem(
        QString::fromStdString(name + "\n" + (type.empty() ? "型不明" : type)));
      item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
      item->setData(Qt::UserRole, QString::fromStdString(name));
      item->setData(Qt::UserRole + 1, QString::fromStdString(type));
      item->setCheckState(isTopicAssigned(name) ? Qt::Checked : Qt::Unchecked);
      topics_->addItem(item);
    }

    loading_topics_ = wasLoadingTopics;
    applyFilter();
    status_->setText(
      QString::number(topics_->count()) + " topics detected");
  }

  void applyFilter()
  {
    const QString keyword = search_->text().trimmed();
    for (int i = 0; i < topics_->count(); ++i) {
      auto * item = topics_->item(i);
      item->setHidden(!keyword.isEmpty() &&
        !item->text().contains(keyword, Qt::CaseInsensitive));
    }
  }

  void startEcho()
  {
    const auto selections = assignedTopicSelections();
    if (selections.empty()) {
      status_->setText("Echoを開始する前に、少なくとも1つのTopicを表示枠へ割り当ててください。");
      return;
    }

    const auto errors = node_->startEcho(selections);
    echo_active_ = errors.size() < selections.size();
    syncTopicCheckStates();
    refreshOutput();

    status_->setText(
      QString("Echo中: %1 topics").arg(selections.size() - errors.size()));
    if (!errors.empty()) {
      QString errorText = "一部の購読に失敗:\n";
      for (const auto & error : errors) {
        errorText += QString::fromStdString(error) + "\n";
      }
      status_->setText(errorText.trimmed());
    }
  }

  void assignTopicToSelectedPane(QListWidgetItem * item)
  {
    if (item == nullptr) return;
    const std::string topic = item->data(Qt::UserRole).toString().toStdString();
    if (topic.empty()) return;

    LayoutNode * pane = findLayoutNode(layout_root_.get(), selected_layout_id_);
    if (pane == nullptr || pane->is_split) {
      status_->setText("トピックを割り当てる表示枠を選択してください。");
      return;
    }

    topic_types_[topic] = item->data(Qt::UserRole + 1).toString().toStdString();
    const bool alreadyInSelectedPane =
      std::find(pane->topics.begin(), pane->topics.end(), topic) != pane->topics.end();

    if (alreadyInSelectedPane) {
      // Clicking a topic already selected in this pane deselects it.
      pane->topics.erase(
        std::remove(pane->topics.begin(), pane->topics.end(), topic),
        pane->topics.end());
      rebuildLayoutWidget();
    } else {
      // If selected in another pane, move it instead of duplicating it.
      removeTopicFromAssignments(layout_root_.get(), topic);
      pane = findLayoutNode(layout_root_.get(), selected_layout_id_);
      if (pane == nullptr || pane->is_split) return;
      pane->topics.push_back(topic);
      rebuildLayoutWidget();
    }

    QString subscriptionErrors;
    if (echo_active_) {
      subscriptionErrors = syncSubscriptionsWithAssignments();
    } else {
      syncTopicCheckStates();
      refreshOutput();
    }

    saveTopicSelection();
    if (!subscriptionErrors.isEmpty()) {
      status_->setText(
        QString("選択変更後、一部の購読処理に失敗しました:\n%1")
          .arg(subscriptionErrors));
    } else if (alreadyInSelectedPane) {
      status_->setText(
        QString("%1 の選択を解除しました。").arg(QString::fromStdString(topic)));
    } else {
      status_->setText(
        QString("%1 を表示枠 %2 に選択しました。")
          .arg(QString::fromStdString(topic)).arg(selected_layout_id_));
    }
  }

  void updateTopicFieldSelector(
    const std::string & topic, TopicDisplayWidgets & view,
    const NumericFields & fields)
  {
    if (fields.empty()) return;

    std::vector<std::string> field_names;
    field_names.reserve(fields.size());
    for (const auto & [field, value] : fields) {
      Q_UNUSED(value);
      field_names.push_back(field);
    }
    if (field_names == view.available_fields) return;
    view.available_fields = std::move(field_names);

    std::string selected = selected_graph_fields_[topic];
    const auto selected_it = fields.find(selected);
    if (selected_it == fields.end()) selected = fields.begin()->first;

    QSignalBlocker blocker(view.field_selector);
    view.field_selector->clear();
    if (fields.size() > 1) {
      for (const auto & [field, value] : fields) {
        Q_UNUSED(value);
        view.field_selector->addItem(QString::fromStdString(field),
          QString::fromStdString(field));
      }
      const int index = view.field_selector->findData(QString::fromStdString(selected));
      view.field_selector->setCurrentIndex(index >= 0 ? index : 0);
      view.field_selector->setVisible(graph_mode_);
    } else {
      view.field_selector->setVisible(false);
      selected = fields.begin()->first;
    }

    selected_graph_fields_[topic] = selected;
    view.graph_view->setSelectedField(selected);
  }

  void appendTopicUpdateToView(
    const std::string & topic, TopicDisplayWidgets & view,
    const TopicUpdate & update)
  {
    if (!update.numeric_fields.empty()) {
      updateTopicFieldSelector(topic, view, update.numeric_fields);
    }
    view.graph_view->addSample(update);
    view.has_received = true;
    view.placeholder.clear();
    view.text_view->setPlaceholderText(QString());

    const QString timestamp = QDateTime::fromMSecsSinceEpoch(
      update.wall_timestamp_ms).toString("HH:mm:ss.zzz");
    const QString entry = QString("[%1]\n%2")
      .arg(timestamp, QString::fromStdString(update.text));
    QScrollBar * scroll_bar = view.text_view->verticalScrollBar();
    const int old_value = scroll_bar->value();
    const bool follow_tail = old_value >= scroll_bar->maximum() - 1;
    view.text_view->appendPlainText(entry);
    if (follow_tail) {
      scroll_bar->setValue(scroll_bar->maximum());
    } else {
      scroll_bar->setValue(std::min(old_value, scroll_bar->maximum()));
    }
    topic_scroll_states_[topic] = {scroll_bar->value(), follow_tail};
  }

  QWidget * createTopicDisplayWidget(const std::string & topic, QWidget * parent)
  {
    auto * card = new QWidget(parent);
    card->setObjectName("topicDisplayCard");
    card->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    auto * card_layout = new QVBoxLayout(card);
    card_layout->setContentsMargins(5, 4, 5, 5);
    card_layout->setSpacing(3);

    auto * header = new QWidget(card);
    header->setFixedHeight(28);
    auto * header_layout = new QHBoxLayout(header);
    header_layout->setContentsMargins(2, 0, 2, 0);
    header_layout->setSpacing(6);

    auto * topic_label = new QLabel(QString::fromStdString(topic), header);
    topic_label->setToolTip(QString::fromStdString(topic));
    topic_label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    header_layout->addWidget(topic_label, 1);

    auto * field_selector = new QComboBox(header);
    field_selector->setMinimumWidth(100);
    field_selector->setMaximumWidth(260);
    field_selector->setFixedHeight(24);
    field_selector->setToolTip("グラフに表示する数値フィールド");
    field_selector->hide();
    header_layout->addWidget(field_selector);

    auto * stack = new QStackedWidget(card);
    auto * text_view = new QPlainTextEdit(stack);
    text_view->setReadOnly(true);
    text_view->setUndoRedoEnabled(false);
    text_view->setLineWrapMode(QPlainTextEdit::NoWrap);
    text_view->setFont(QFont("monospace", 9));
    text_view->setMaximumBlockCount(50000);
    text_view->setPlaceholderText(node_->hasSubscription(topic) ?
      "受信待ち..." : "未購読（echo開始で受信します）");

    auto * graph_view = new NumericGraphWidget(stack);
    graph_view->setStatusMessage(node_->hasSubscription(topic) ?
      "数値データ受信待ち..." : "未購読（echo開始で受信します）");
    stack->addWidget(text_view);
    stack->addWidget(graph_view);
    stack->setCurrentWidget(graph_mode_ ?
      static_cast<QWidget *>(graph_view) : static_cast<QWidget *>(text_view));

    card_layout->addWidget(header);
    card_layout->addWidget(stack, 1);

    TopicDisplayWidgets view;
    view.card = card;
    view.field_selector = field_selector;
    view.stack = stack;
    view.text_view = text_view;
    view.graph_view = graph_view;
    topic_display_widgets_[topic] = view;

    connect(field_selector, QOverload<int>::of(&QComboBox::currentIndexChanged),
      this, [this, topic, field_selector](int index) {
        const QString field = field_selector->itemData(index).toString();
        if (field.isEmpty()) return;
        selected_graph_fields_[topic] = field.toStdString();
        const auto view_it = topic_display_widgets_.find(topic);
        if (view_it != topic_display_widgets_.end()) {
          view_it->second.graph_view->setSelectedField(field.toStdString());
        }
      });

    const auto history_it = topic_histories_.find(topic);
    if (history_it != topic_histories_.end()) {
      QStringList entries;
      for (const auto & update : history_it->second) {
        if (!update.numeric_fields.empty()) {
          updateTopicFieldSelector(topic, topic_display_widgets_.at(topic),
            update.numeric_fields);
        }
        graph_view->addSample(update);
        const QString timestamp = QDateTime::fromMSecsSinceEpoch(
          update.wall_timestamp_ms).toString("HH:mm:ss.zzz");
        entries.append(QString("[%1]\n%2")
          .arg(timestamp, QString::fromStdString(update.text)));
      }
      if (!entries.isEmpty()) {
        text_view->setPlainText(entries.join("\n"));
        QScrollBar * scroll_bar = text_view->verticalScrollBar();
        const auto scroll_it = topic_scroll_states_.find(topic);
        if (scroll_it != topic_scroll_states_.end() && !scroll_it->second.second) {
          scroll_bar->setValue(std::min(scroll_it->second.first, scroll_bar->maximum()));
        } else {
          scroll_bar->setValue(scroll_bar->maximum());
        }
        topic_scroll_states_[topic] = {
          scroll_bar->value(), scroll_bar->value() >= scroll_bar->maximum() - 1};
        topic_display_widgets_[topic].has_received = true;
        text_view->setPlaceholderText(QString());
      }
    }
    return card;
  }

  void setGraphMode(bool enabled)
  {
    graph_mode_ = enabled;
    if (display_mode_button_ != nullptr) {
      display_mode_button_->setText(
        enabled ? "表示方式：グラフ" : "表示方式：テキスト");
    }
    for (auto & [topic, view] : topic_display_widgets_) {
      Q_UNUSED(topic);
      view.stack->setCurrentWidget(enabled ?
        static_cast<QWidget *>(view.graph_view) :
        static_cast<QWidget *>(view.text_view));
      view.field_selector->setVisible(
        enabled && view.available_fields.size() > 1);
    }
  }

  QWidget * createLayoutWidget(LayoutNode & node)
  {
    if (node.is_split && node.first && node.second) {
      auto * splitter = new QSplitter(node.orientation);
      splitter->setChildrenCollapsible(false);
      splitter->setHandleWidth(6);
      splitter_widgets_[node.id] = splitter;
      splitter->addWidget(createLayoutWidget(*node.first));
      splitter->addWidget(createLayoutWidget(*node.second));

      const int ratio = node.ratio;
      QTimer::singleShot(0, splitter, [splitter, ratio]() {
        if (splitter->count() == 2) {
          splitter->setSizes({ratio, 1000 - ratio});
        }
      });
      connect(splitter, &QSplitter::splitterMoved, this,
        [this, id = node.id](int, int) {
          updateSplitRatio(id);
          if (layout_save_timer_ != nullptr) layout_save_timer_->start();
        });
      return splitter;
    }

    auto * pane = new QWidget();
    pane->setObjectName("topicPane");
    pane->setMinimumSize(100, 80);
    pane_widgets_[node.id] = pane;

    auto * grid = new QGridLayout(pane);
    grid->setContentsMargins(4, 4, 4, 4);
    grid->setHorizontalSpacing(6);
    grid->setVerticalSpacing(6);

    if (node.topics.empty()) {
      auto * empty_label = new QLabel("Topic未割り当て", pane);
      empty_label->setAlignment(Qt::AlignCenter);
      grid->addWidget(empty_label, 0, 0);
    } else {
      const int columns = std::max(1, static_cast<int>(
        std::ceil(std::sqrt(static_cast<double>(node.topics.size())))));
      for (size_t index = 0; index < node.topics.size(); ++index) {
        const std::string & topic = node.topics[index];
        auto * topic_card = createTopicDisplayWidget(topic, pane);
        const int row = static_cast<int>(index) / columns;
        const int column = static_cast<int>(index) % columns;
        grid->addWidget(topic_card, row, column);
        grid->setColumnStretch(column, 1);
        grid->setRowStretch(row, 1);
      }
    }

    // Select this pane from clicks anywhere in it, including each Topic view.
    // Do not consume events so text scrolling, selection, and field selection work.
    pane->installEventFilter(this);
    for (QWidget * child : pane->findChildren<QWidget *>()) {
      child->installEventFilter(this);
    }
    return pane;
  }

  void rebuildLayoutWidget()
  {
    captureSplitterRatios();
    for (const auto & [topic, view] : topic_display_widgets_) {
      QScrollBar * scroll_bar = view.text_view->verticalScrollBar();
      topic_scroll_states_[topic] = {
        scroll_bar->value(), scroll_bar->value() >= scroll_bar->maximum() - 1};
    }
    splitter_widgets_.clear();
    pane_widgets_.clear();
    topic_display_widgets_.clear();

    if (layout_widget_ != nullptr) {
      output_layout_->removeWidget(layout_widget_);
      delete layout_widget_;
      layout_widget_ = nullptr;
    }

    if (!layout_root_) {
      layout_root_ = std::make_unique<LayoutNode>();
      layout_root_->id = next_layout_id_++;
    }
    if (findLayoutNode(layout_root_.get(), selected_layout_id_) == nullptr ||
      findLayoutNode(layout_root_.get(), selected_layout_id_)->is_split) {
      selected_layout_id_ = firstLeafId(layout_root_.get());
    }

    layout_widget_ = createLayoutWidget(*layout_root_);
    output_layout_->addWidget(layout_widget_, 1);

    std::set<std::string> active_topics;
    for (const auto & [topic, view] : topic_display_widgets_) {
      Q_UNUSED(view);
      active_topics.insert(topic);
    }
    for (auto it = topic_histories_.begin(); it != topic_histories_.end();) {
      if (active_topics.count(it->first) == 0) it = topic_histories_.erase(it);
      else ++it;
    }
    for (auto it = topic_history_bytes_.begin(); it != topic_history_bytes_.end();) {
      if (active_topics.count(it->first) == 0) it = topic_history_bytes_.erase(it);
      else ++it;
    }
    for (auto it = selected_graph_fields_.begin(); it != selected_graph_fields_.end();) {
      if (active_topics.count(it->first) == 0) it = selected_graph_fields_.erase(it);
      else ++it;
    }
    for (auto it = topic_scroll_states_.begin(); it != topic_scroll_states_.end();) {
      if (active_topics.count(it->first) == 0) it = topic_scroll_states_.erase(it);
      else ++it;
    }

    updatePaneSelectionVisuals();
    refreshOutput();
  }

  LayoutNode * findLayoutNode(LayoutNode * node, int id) const
  {
    if (node == nullptr) return nullptr;
    if (node->id == id) return node;
    if (!node->is_split) return nullptr;
    if (auto * found = findLayoutNode(node->first.get(), id)) return found;
    return findLayoutNode(node->second.get(), id);
  }

  std::unique_ptr<LayoutNode> * findLayoutNodeSlot(
    std::unique_ptr<LayoutNode> & node, int id)
  {
    if (!node) return nullptr;
    if (node->id == id) return &node;
    if (!node->is_split) return nullptr;
    if (auto * found = findLayoutNodeSlot(node->first, id)) return found;
    return findLayoutNodeSlot(node->second, id);
  }

  int firstLeafId(const LayoutNode * node) const
  {
    if (node == nullptr) return -1;
    if (!node->is_split || !node->first || !node->second) return node->id;
    const int first = firstLeafId(node->first.get());
    return first >= 0 ? first : firstLeafId(node->second.get());
  }

  bool removeLeafAndCollapse(std::unique_ptr<LayoutNode> & node, int id)
  {
    if (!node || !node->is_split) return false;
    if (node->first && !node->first->is_split && node->first->id == id) {
      node = std::move(node->second);
      return true;
    }
    if (node->second && !node->second->is_split && node->second->id == id) {
      node = std::move(node->first);
      return true;
    }
    if (removeLeafAndCollapse(node->first, id)) return true;
    return removeLeafAndCollapse(node->second, id);
  }

  void splitSelectedPane(Qt::Orientation orientation)
  {
    auto * slot = findLayoutNodeSlot(layout_root_, selected_layout_id_);
    if (slot == nullptr || !*slot || (*slot)->is_split) {
      status_->setText("分割する表示枠を選択してください。");
      return;
    }

    auto oldLeaf = std::move(*slot);
    auto split = std::make_unique<LayoutNode>();
    split->id = next_layout_id_++;
    split->is_split = true;
    split->orientation = orientation;
    split->ratio = 500;
    split->first = std::move(oldLeaf);
    split->second = std::make_unique<LayoutNode>();
    split->second->id = next_layout_id_++;
    selected_layout_id_ = split->second->id;
    *slot = std::move(split);

    rebuildLayoutWidget();
    saveTopicSelection();
    status_->setText(
      orientation == Qt::Horizontal ?
      "表示枠を左右に分割しました。新しい枠を選択できます。" :
      "表示枠を上下に分割しました。新しい枠を選択できます。");
  }

  void deleteSelectedPane()
  {
    if (!layout_root_) return;
    if (!layout_root_->is_split) {
      layout_root_->topics.clear();
      rebuildLayoutWidget();
      QString errors;
      if (echo_active_) errors = syncSubscriptionsWithAssignments();
      else syncTopicCheckStates();
      saveTopicSelection();
      status_->setText(errors.isEmpty() ?
        "最後の表示枠は残し、割り当てTopicを解除しました。" :
        "割当解除後の購読処理でエラーが発生しました:\n" + errors);
      return;
    }

    if (!removeLeafAndCollapse(layout_root_, selected_layout_id_)) {
      status_->setText("削除する表示枠を選択してください。");
      return;
    }
    selected_layout_id_ = firstLeafId(layout_root_.get());
    rebuildLayoutWidget();
    QString errors;
    if (echo_active_) errors = syncSubscriptionsWithAssignments();
    else syncTopicCheckStates();
    saveTopicSelection();
    status_->setText(errors.isEmpty() ?
      "表示枠を削除しました。" :
      "表示枠削除後の購読処理でエラーが発生しました:\n" + errors);
  }

  void selectLayoutPane(int id)
  {
    LayoutNode * node = findLayoutNode(layout_root_.get(), id);
    if (node == nullptr || node->is_split) return;
    selected_layout_id_ = id;
    updatePaneSelectionVisuals();
    saveTopicSelection();
  }

  void updatePaneSelectionVisuals()
  {
    for (const auto & [id, pane] : pane_widgets_) {
      pane->setStyleSheet(id == selected_layout_id_ ?
        "QWidget#topicPane { border: 2px solid #3574B8; }" :
        "QWidget#topicPane { border: 1px solid #888888; }");
    }
  }

  void updateSplitRatio(int id)
  {
    LayoutNode * node = findLayoutNode(layout_root_.get(), id);
    auto splitterIt = splitter_widgets_.find(id);
    if (node == nullptr || !node->is_split || splitterIt == splitter_widgets_.end()) return;
    const QList<int> sizes = splitterIt->second->sizes();
    if (sizes.size() != 2 || sizes[0] + sizes[1] <= 0) return;
    node->ratio = std::clamp(
      static_cast<int>(1000LL * sizes[0] / (sizes[0] + sizes[1])), 100, 900);
  }

  void captureSplitterRatios()
  {
    for (const auto & [id, splitter] : splitter_widgets_) {
      Q_UNUSED(splitter);
      updateSplitRatio(id);
    }
  }

  void refreshOutput()
  {
    // Drain all received samples in batches on the GUI thread. ROS callbacks
    // only enqueue data; they never access Qt widgets.
    const auto updates = node_->takePendingUpdates();
    for (const auto & update : updates) {
      auto & history = topic_histories_[update.topic];
      auto & history_bytes = topic_history_bytes_[update.topic];
      history.push_back(update);
      history_bytes += estimateTopicUpdateBytes(update);
      while (history.size() > 1 &&
        (history.size() > kTopicHistoryLimit ||
        history_bytes > kTopicHistoryByteLimit))
      {
        history_bytes -= estimateTopicUpdateBytes(history.front());
        history.pop_front();
      }

      const auto view_it = topic_display_widgets_.find(update.topic);
      if (view_it != topic_display_widgets_.end()) {
        appendTopicUpdateToView(update.topic, view_it->second, update);
      }
    }

    for (auto & [topic, view] : topic_display_widgets_) {
      if (view.has_received) continue;
      const bool subscribed = node_->hasSubscription(topic);
      const QString text_placeholder = subscribed ?
        "受信待ち..." : "未購読（echo開始で受信します）";
      const QString graph_placeholder = subscribed ?
        "数値データ受信待ち..." : "未購読（echo開始で受信します）";
      if (view.placeholder != text_placeholder) {
        view.placeholder = text_placeholder;
        view.text_view->setPlaceholderText(text_placeholder);
        view.graph_view->setStatusMessage(graph_placeholder);
      }
    }
  }

  void normalizeTopicAssignments(LayoutNode * node, std::set<std::string> & seen)
  {
    if (node == nullptr) return;
    if (node->is_split) {
      normalizeTopicAssignments(node->first.get(), seen);
      normalizeTopicAssignments(node->second.get(), seen);
      return;
    }

    std::vector<std::string> uniqueTopics;
    for (const auto & topic : node->topics) {
      if (!topic.empty() && seen.insert(topic).second) {
        uniqueTopics.push_back(topic);
      }
    }
    node->topics = std::move(uniqueTopics);
  }

  void removeTopicFromAssignments(LayoutNode * node, const std::string & topic)
  {
    if (node == nullptr) return;
    if (node->is_split) {
      removeTopicFromAssignments(node->first.get(), topic);
      removeTopicFromAssignments(node->second.get(), topic);
      return;
    }
    node->topics.erase(
      std::remove(node->topics.begin(), node->topics.end(), topic), node->topics.end());
  }

  void clearTopicAssignments(LayoutNode * node)
  {
    if (node == nullptr) return;
    if (node->is_split) {
      clearTopicAssignments(node->first.get());
      clearTopicAssignments(node->second.get());
      return;
    }
    node->topics.clear();
  }

  void collectAssignedTopics(
    const LayoutNode * node, std::vector<std::string> & topics) const
  {
    if (node == nullptr) return;
    if (node->is_split) {
      collectAssignedTopics(node->first.get(), topics);
      collectAssignedTopics(node->second.get(), topics);
      return;
    }
    for (const auto & topic : node->topics) topics.push_back(topic);
  }

  std::vector<std::pair<std::string, std::string>> assignedTopicSelections() const
  {
    std::vector<std::string> assigned;
    collectAssignedTopics(layout_root_.get(), assigned);
    std::set<std::string> seen;
    std::vector<std::pair<std::string, std::string>> selections;
    for (const auto & topic : assigned) {
      if (topic.empty() || !seen.insert(topic).second) continue;
      const auto typeIt = topic_types_.find(topic);
      selections.emplace_back(
        topic, typeIt == topic_types_.end() ? std::string() : typeIt->second);
    }
    return selections;
  }

  bool isTopicAssignedInLayout(const LayoutNode * node, const std::string & topic) const
  {
    if (node == nullptr) return false;
    if (node->is_split) {
      return isTopicAssignedInLayout(node->first.get(), topic) ||
             isTopicAssignedInLayout(node->second.get(), topic);
    }
    return std::find(node->topics.begin(), node->topics.end(), topic) != node->topics.end();
  }

  bool isTopicAssigned(const std::string & topic) const
  {
    return isTopicAssignedInLayout(layout_root_.get(), topic);
  }

  void syncTopicCheckStates()
  {
    const bool wasLoading = loading_topics_;
    loading_topics_ = true;
    for (int i = 0; i < topics_->count(); ++i) {
      auto * item = topics_->item(i);
      const std::string topic = item->data(Qt::UserRole).toString().toStdString();
      item->setCheckState(isTopicAssigned(topic) ? Qt::Checked : Qt::Unchecked);
    }
    loading_topics_ = wasLoading;
  }

  QString syncSubscriptionsWithAssignments()
  {
    if (!echo_active_) {
      syncTopicCheckStates();
      return {};
    }

    const auto selections = assignedTopicSelections();
    std::set<std::string> desired;
    for (const auto & selection : selections) desired.insert(selection.first);

    for (const auto & subscribed : node_->subscriptionNames()) {
      if (desired.count(subscribed) == 0) node_->removeSubscription(subscribed);
    }

    QString errors;
    for (const auto & [topic, type] : selections) {
      if (node_->hasSubscription(topic)) continue;
      if (type.empty()) {
        errors += QString("%1: Topic型がまだ検出されていません。\n")
          .arg(QString::fromStdString(topic));
        continue;
      }
      const std::string error = node_->addSubscription(topic, type);
      if (!error.empty()) {
        errors += QString::fromStdString(error) + "\n";
      }
    }

    syncTopicCheckStates();
    refreshOutput();
    return errors.trimmed();
  }

  std::shared_ptr<EchoNode> node_;
  bool loading_topics_{false};
  bool echo_active_{false};
  std::unique_ptr<LayoutNode> layout_root_;
  int next_layout_id_{1};
  int selected_layout_id_{1};
  TopicListWidget * topics_{};
  QWidget * topics_panel_{};
  QLineEdit * search_{};
  QWidget * output_container_{};
  QVBoxLayout * output_layout_{};
  QWidget * layout_widget_{};
  std::map<int, QWidget *> pane_widgets_;
  std::map<int, QSplitter *> splitter_widgets_;
  std::map<std::string, TopicDisplayWidgets> topic_display_widgets_;
  std::map<std::string, std::deque<TopicUpdate>> topic_histories_;
  std::map<std::string, size_t> topic_history_bytes_;
  std::map<std::string, std::string> selected_graph_fields_;
  std::map<std::string, std::pair<int, bool>> topic_scroll_states_;
  std::map<std::string, std::string> topic_types_;
  QPushButton * display_mode_button_{};
  bool graph_mode_{false};
  QLabel * status_{};
  QTimer * timer_{};
  QTimer * layout_save_timer_{};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  QApplication app(argc, argv);
  auto node = std::make_shared<EchoNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  MainWindow window(node);
  window.show();
  const int result = app.exec();

  executor.cancel();
  if (spin_thread.joinable()) spin_thread.join();
  rclcpp::shutdown();
  return result;
}
