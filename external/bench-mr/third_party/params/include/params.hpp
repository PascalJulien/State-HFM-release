#pragma once

#include <string>
#include <unordered_map>
#include <variant>

#include <nlohmann/json.hpp>

namespace params {
class P0 {
public:
  inline const std::string &name() const { return name_; }

  virtual void serialize(nlohmann::json &j) const { j[name_] = {}; }
  virtual void serialize(std::ostream &stream) const { stream << name_; }

  friend std::ostream &operator<<(std::ostream &stream, const P0 &p) {
    return stream << p.name_;
  }

  virtual void load(const nlohmann::json &j) = 0;

protected:
  std::string name_{""};
  P0(const std::string &name) : name_(name) {}
};

struct Group {
public:
  Group(const std::string &name, Group *parent = nullptr) : name_(name) {
    if (parent != nullptr)
      parent->subgroups_[name] = this;
  }
  Group(const char *name, Group *parent = nullptr) : name_(name) {
    if (parent != nullptr)
      parent->subgroups_[name] = this;
  }

  std::unordered_map<std::string, P0 *> &members() { return members_; }
  const std::unordered_map<std::string, P0 *> &members() const {
    return members_;
  }

  std::unordered_map<std::string, Group *> &subgroups() { return subgroups_; }
  const std::unordered_map<std::string, Group *> &subgroups() const {
    return subgroups_;
  }

  friend std::ostream &operator<<(std::ostream &stream, const Group &g) {
    stream << '<' << g.name_ << '>' << std::endl;
    for (const auto &member : g.members_) {
      stream << '\t';
      member.second->serialize(stream);
      stream << std::endl;
    }
    for (const auto &member : g.subgroups_)
      stream << *member.second << std::endl;
    stream << "</" << g.name_ << '>';
    return stream;
  }

  const std::string &name() const { return name_; }

  void load(const nlohmann::json &j) {
    if (j.find(name_) == j.end())
      return;
    const auto subj = j[name_];
    for (const auto &member : members_)
      member.second->load(subj);
    for (const auto &member : subgroups_)
      member.second->load(subj);
  }

private:
  std::string name_;
  std::unordered_map<std::string, Group *> subgroups_;

  std::unordered_map<std::string, P0 *> members_;
};

template <typename T> class Property : public P0 {
public:
  explicit Property(Group *group = nullptr) : P0(""), group_(group) {
    if (group != nullptr)
      group->members()[name_] = this;
  }

  explicit Property(const T &value, const std::string &name = "",
                    Group *group = nullptr)
      : P0(name), value_(value), group_(group) {
    if (group != nullptr)
      group->members()[name_] = this;
  }

  inline operator T &() { return value_; }
  inline operator const T &() const { return value_; }

  inline const T &value() const { return value_; }
  inline T &value() { return value_; }

  Property<T> &operator=(const T &v) {
    value_ = v;
    return *this;
  }

  bool operator==(const Property<T> &other) const {
    return name_ == other.name_ && value_ == other.value_;
  }

  friend std::ostream &operator<<(std::ostream &stream, const Property<T> &p) {
    if (p.name_.empty())
      return stream << p.value_;
    return stream << p.name_ << '=' << p.value_;
  }

  void serialize(nlohmann::json &j) const override { j[name_] = value_; }
  virtual void serialize(std::ostream &stream) const {
    stream << name_ << '=' << value_;
  }

  void load(const nlohmann::json &j) override {
    if (j.find(name_) == j.end())
      return;
    value_ = j[name_].get<T>();
  }

protected:
  T value_{};
  Group *group_{nullptr};
};

template <typename U>
inline std::ostream &operator<<(std::ostream &stream, const std::vector<U> &p) {
  stream << '[';
  for (auto i = 0u; i < p.size(); ++i) {
    stream << p[i];
    if (i < p.size() - 1)
      stream << ", ";
  }
  return stream << ']';
}

using nlohmann::json;
template <typename T> void to_json(json &j, const Property<T> &p) {
  j[p.name()] = p.value();
}

inline void to_json(json &j, const Group &g) {
  auto &j_ = j[g.name()];
  for (const auto &subgroup : g.subgroups())
    to_json(j_, *subgroup.second);
  for (const auto &member : g.members()) {
    member.second->serialize(j_);
  }
}

} // namespace params
