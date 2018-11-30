#include <algorithm>

#include "scenegraph.h"

using namespace std;
using namespace std::tr1;

bool SgTransformNode::accept(SgNodeVisitor& visitor) {
  if (!visitor.visit(*this))
    return false;
  for (int i = 0, n = children_.size(); i < n; ++i) {
    if (!children_[i]->accept(visitor))
      return false;
  }
  return visitor.postVisit(*this);
}

void SgTransformNode::addChild(shared_ptr<SgNode> child) {
  children_.push_back(child);
}

void SgTransformNode::removeChild(shared_ptr<SgNode> child) {
  children_.erase(find(children_.begin(), children_.end(), child));
}

bool SgShapeNode::accept(SgNodeVisitor& visitor) {
  if (!visitor.visit(*this))
    return false;
  return visitor.postVisit(*this);
}

class RbtAccumVisitor : public SgNodeVisitor {
protected:
  vector<RigTForm> rbtStack_;
  SgTransformNode& target_;
  bool found_;
public:
  RbtAccumVisitor(SgTransformNode& target)
    : target_(target)
    , found_(false) {}

  const RigTForm getAccumulatedRbt(int offsetFromStackTop = 0) {
	  RigTForm accumulated = RigTForm::identity();
	  for (vector<RigTForm>::iterator i = rbtStack_.begin();
		   i != rbtStack_.end();
		   i++) accumulated = accumulated * *i;

  }

  virtual bool visit(SgTransformNode& node) {
	  rbtStack_.push_back(node.getRbt());
	  found_ = node == target_;
	  return !found_;
  }

  virtual bool postVisit(SgTransformNode& node) {
	  rbtStack_.pop_back();
  }
};

RigTForm getPathAccumRbt(
  shared_ptr<SgTransformNode> source,
  shared_ptr<SgTransformNode> destination,
  int offsetFromDestination) {

  RbtAccumVisitor accum(*destination);
  source->accept(accum);
  return accum.getAccumulatedRbt(offsetFromDestination);
}

class DescendantVisitor : public SgNodeVisitor {
protected:
  SgTransformNode& target_;
  bool found_;
public:
  DescendantVisitor(SgTransformNode& target)
    : target_(target)
    , found_(false) {}

   const bool getFound() {
      return found_;
   }

  virtual bool visit(SgTransformNode& node) {
	  if (node == target_)
		  found_ = true;

	  return !found_;
  }
};

bool checkForDescendant(
  shared_ptr<SgTransformNode> source,
  shared_ptr<SgTransformNode> destination) {

  DescendantVisitor dv(*destination);
  source->accept(dv);
  return dv.getFound();
}
