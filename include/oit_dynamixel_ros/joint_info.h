#ifndef ___JOINT_INFO_H___
#define ___JOINT_INFO_H___
#include <string>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/random_access_index.hpp>
#include <boost/multi_index/random_access_index.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/range/algorithm/for_each.hpp>
#include <boost/thread/mutex.hpp>

typedef struct tagJointInfo
{
  int id;
  std::string name;
  double max_velocity;
  tagJointInfo(int id_ = -1, const std::string &name_ = "") : id(id_), name(name_) {}
  virtual ~tagJointInfo() {}
  std::string toString() const;
  typedef boost::shared_ptr<tagJointInfo> Ptr;
  typedef boost::shared_ptr<const tagJointInfo> ConstPtr;
  struct tagId
  {
  };
  struct tagName
  {
  };
  struct tagVector
  {
  };

  typedef boost::multi_index_container<
      Ptr,
      boost::multi_index::indexed_by<
          boost::multi_index::ordered_unique<boost::multi_index::tag<tagId>, boost::multi_index::member<tagJointInfo, int, &tagJointInfo::id>>,
          boost::multi_index::ordered_unique<boost::multi_index::tag<tagName>, boost::multi_index::member<tagJointInfo, std::string, &tagJointInfo::name>>,
          boost::multi_index::random_access<boost::multi_index::tag<tagVector>>>>
      Container;
} JointInfo;

#endif
