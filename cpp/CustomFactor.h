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

/**
 * @brief Custom factor that takes a std::function as the error
 * @addtogroup nonlinear
 * \nosubgrouping
 *
 * This factor is mainly for creating a custom factor in Python.
 */
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
   * @param keys keys of the variables
   * @param errorFunction the error functional
   */
  CustomFactor(const SharedNoiseModel& noiseModel, const KeyVector& keys, const CustomErrorFunction& errorFunction) :
      Base(noiseModel, keys) {
    this->errorFunction = errorFunction;
  }

  ~CustomFactor() override = default;

  /** Calls the errorFunction closure, which is a std::function object
    * One can check if a derivative is needed in the errorFunction by checking the length of Jacobian array
  */
  Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const override;

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
