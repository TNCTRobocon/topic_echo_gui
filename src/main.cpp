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
#include <QStyleOptionViewItem>
#include <QPushButton>
#include <QSaveFile>
#include <QSplitter>
#include <QStyle>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
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
  std::function<void(QListWidgetItem *)> onTopicNameClicked;

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
    if (swallow_check_click_ ||
      (event->button() == Qt::LeftButton && isCheckIndicatorAt(event->pos())))
    {
      swallow_check_click_ = false;
      event->accept();
      return;
    }

    QListWidgetItem * clickedItem = itemAt(event->pos());
    const bool nameClick =
      event->button() == Qt::LeftButton && clickedItem != nullptr;
    QListWidget::mouseReleaseEvent(event);
    if (nameClick && onTopicNameClicked) {
      onTopicNameClicked(clickedItem);
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
            updateLatest(name, formatJoy(*msg));
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
    std::lock_guard<std::mutex> lock(latest_mutex_);
    latest_.erase(name);
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
    std::lock_guard<std::mutex> lock(latest_mutex_);
    latest_.clear();
  }
  using Snapshot = std::map<std::string, std::pair<uint64_t, std::string>>;

  Snapshot snapshot() const
  {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    return latest_;
  }

private:
  void updateLatest(const std::string & topic, const std::string & text)
  {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    auto & slot = latest_[topic];
    ++slot.first;
    slot.second = text;
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
        updateLatest(topic, renderMessage(members, memory, 0));
      } catch (...) {
        members->fini_function(memory);
        std::free(memory);
        throw;
      }
      members->fini_function(memory);
      std::free(memory);
    } catch (const std::exception & e) {
      updateLatest(topic, std::string("表示変換エラー: ") + e.what());
    }
  }

  rclcpp::QoS qos_;
  std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  mutable std::mutex latest_mutex_;
  Snapshot latest_;
};
class MainWindow final : public QMainWindow
{
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

    auto * central = new QWidget(this);
    auto * root = new QVBoxLayout(central);
    auto * toolbar = new QHBoxLayout();
    auto * content = new QHBoxLayout();
    auto * layoutToolbar = new QHBoxLayout();

    auto * refresh = new QPushButton("更新");
    auto * clearSel = new QPushButton("すべての割当を解除");
    auto * toggleTopics = new QPushButton("Topic一覧を隠す");
    auto * start = new QPushButton("ECHO開始");
    auto * stop = new QPushButton("Echo停止");
    auto * clearOutput = new QPushButton("表示クリア");
    auto * splitHorizontal = new QPushButton("左右に分割");
    auto * splitVertical = new QPushButton("上下に分割");
    auto * deletePane = new QPushButton("表示枠を削除");
    search_ = new QLineEdit();
    search_->setPlaceholderText("Topic名または型を検索");

    toolbar->addWidget(refresh);
    toolbar->addWidget(clearSel);
    toolbar->addWidget(toggleTopics);
    toolbar->addWidget(start);
    toolbar->addWidget(stop);
    toolbar->addWidget(clearOutput);
    toolbar->addSpacing(12);
    toolbar->addWidget(new QLabel("検索:"));
    toolbar->addWidget(search_, 1);

    layoutToolbar->addWidget(new QLabel("表示枠操作:"));
    layoutToolbar->addWidget(splitHorizontal);
    layoutToolbar->addWidget(splitVertical);
    layoutToolbar->addWidget(deletePane);
    layoutToolbar->addWidget(new QLabel("表示枠内をクリックして選択し、左のTopic名をクリックして割り当て・解除します（チェックは表示枠への割当状態）。"));
    layoutToolbar->addStretch(1);

    topics_ = new TopicListWidget();
    topics_->setSelectionMode(QAbstractItemView::NoSelection);
    topics_->onTopicNameClicked = [this](QListWidgetItem * item) {
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
    left->addWidget(new QLabel("Topic一覧（名前をクリック：選択・解除／チェック：選択状態）"));
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
      for (auto & [id, editor] : output_editors_) {
        Q_UNUSED(id);
        editor->clear();
      }
      syncTopicCheckStates();
      refreshOutput();
      status_->setText("Echo停止中");
    });
    connect(clearOutput, &QPushButton::clicked, this, [this]() {
      for (auto & [topic, editor] : output_editors_) {
        Q_UNUSED(topic);
        editor->clear();
      }
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
    refreshTopics();
    rebuildLayoutWidget();
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
    for (auto & [id, editor] : output_editors_) {
      Q_UNUSED(id);
      editor->clear();
    }
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
    pane_topics_[node.id] = node.topics;

    auto * layout = new QVBoxLayout(pane);
    layout->setContentsMargins(4, 4, 4, 4);
    layout->setSpacing(0);

    auto * editor = new QPlainTextEdit(pane);
    editor->setReadOnly(true);
    editor->setLineWrapMode(QPlainTextEdit::NoWrap);
    editor->setFont(QFont("monospace", 10));
    output_editors_[node.id] = editor;
    layout->addWidget(editor, 1);

    // Select this pane from clicks anywhere in it, including the text viewport
    // and scrollbars. Do not consume the event so normal editor interaction works.
    pane->installEventFilter(this);
    for (QWidget * child : pane->findChildren<QWidget *>()) {
      child->installEventFilter(this);
    }
    return pane;
  }

  void rebuildLayoutWidget()
  {
    captureSplitterRatios();
    splitter_widgets_.clear();
    pane_widgets_.clear();
    output_editors_.clear();
    pane_topics_.clear();

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
    const auto snapshot = node_->snapshot();
    for (const auto & [paneId, editor] : output_editors_) {
      const auto assignedIt = pane_topics_.find(paneId);
      QString rendered;
      if (assignedIt != pane_topics_.end()) {
        for (const auto & topic : assignedIt->second) {
          if (!rendered.isEmpty()) rendered += "\n";
          rendered += QString::fromStdString(topic) + "\n";
          rendered += QString(48, '-') + "\n";

          const auto valueIt = snapshot.find(topic);
          if (!node_->hasSubscription(topic)) {
            rendered += "未購読（Echo開始で受信します）\n";
          } else if (valueIt == snapshot.end()) {
            rendered += "受信待ち...\n";
          } else {
            rendered += QString::fromStdString(valueIt->second.second);
            if (!rendered.endsWith('\n')) rendered += "\n";
          }
          rendered += "\n";
        }
      }

      if (editor->toPlainText() != rendered) {
        editor->setPlainText(rendered);
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
  std::map<int, QPlainTextEdit *> output_editors_;
  std::map<int, std::vector<std::string>> pane_topics_;
  std::map<int, QSplitter *> splitter_widgets_;
  std::map<std::string, std::string> topic_types_;
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
