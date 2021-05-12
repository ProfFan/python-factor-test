//
// Created by fan on 5/12/21.
//

#include "CustomFactor.h"

namespace gtsam_example {

Vector CustomFactor::unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H) const {
  if(this->active(x)) {
    if(H) {
      return this->errorFunction(*this, x, H.get_ptr());
    } else {
      JacobianVector dummy;
      return this->errorFunction(*this, x, &dummy);
    }
  } else {
    return Vector::Zero(this->dim());
  }
}

}
