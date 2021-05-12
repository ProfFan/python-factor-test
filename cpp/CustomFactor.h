//
// Created by fan on 5/12/21.
//

#ifndef GTSAM_EXAMPLE_CUSTOMFACTOR_H
#define GTSAM_EXAMPLE_CUSTOMFACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>

using namespace gtsam;

namespace gtsam_example {

typedef std::vector<Matrix> JacobianVector;

class CustomFactor;

typedef std::function<Vector(const CustomFactor&, const Values&, const JacobianVector*)> CustomErrorFunction;

class CustomFactor: public NoiseModelFactor {
public:
   CustomErrorFunction errorFunction;

protected:

  typedef NoiseModelFactor Base;
  typedef CustomFactor This;

public:

  /**
   * Default Constructor for I/O
   */
  CustomFactor() = default;

  /**
   * Constructor
   * @param noiseModel shared pointer to noise model
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   */
  CustomFactor(const SharedNoiseModel& noiseModel, const KeyVector& keys, const CustomErrorFunction& errorFunction) :
      Base(noiseModel, keys) {
    this->errorFunction = errorFunction;
  }

  ~CustomFactor() override = default;

  /** Calls the 2-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const override {
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

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("CustomFactor",
                                        boost::serialization::base_object<Base>(*this));
  }
};

};

#endif //GTSAM_EXAMPLE_CUSTOMFACTOR_H
