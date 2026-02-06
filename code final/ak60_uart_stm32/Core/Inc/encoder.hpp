#ifndef ROBOTLIB_ENCODER_HPP_
#define ROBOTLIB_ENCODER_HPP_

/**
 * @brief Encoder Class
 */
class Encoder
{
private:
  TIM_HandleTypeDef *henc;     /**< Encoder Timer Handle */
  int16_t cpr;                 /**< Count per revolution */
  uint16_t sample_time;        /** Sample time for calculating omega */
  float omega;                 /**< Angular velocity (rad/s) */
  int32_t count_aggregate = 0; /**< Total count from begining */
  int32_t prevCount = 0;       /**< Count of each iteration before reset */
  float calibration = 0.0f;    /**< Calibration value */
  uint32_t last_reset_time;    /**< Last reset time */

public:
  /**
   * @brief Default Constructor
   */
  Encoder() : henc(nullptr) {}

  /**
   * @brief Copy Constructor
   */
  Encoder(const Encoder &) = default;

  /**
   * @brief Constructor
   * @param _henc Encoder Timer Handle
   * @param _cpr Count per revolution
   * @param _sample_time Sample time for calculating omega
   */
  Encoder(TIM_HandleTypeDef *_henc, int16_t _cpr, uint16_t _sample_time = 1)
      : henc(_henc), cpr(_cpr), sample_time(_sample_time) {}

  /**
   * @brief Destructor
   */
  ~Encoder() = default;

  /**
   * @brief Initialize Encoder Timer
   */
  bool init(void);

  /**
   * @brief Get Encoder Count
   */
  int32_t get_count(void);

  /**
   * @brief Reset Encoder Count
   */
  void reset_encoder_count(void);

  /**
   * @brief Get Angular Velocity
   */
  float get_omega(void);

  /**
   * @brief Get Encoder Count Aggregate
   */
  int32_t get_count_aggregate(void);

  /**
   * @brief Reset Count Aggregate
   */
  void reset_count_aggregate(void) { count_aggregate = 0; }

  void set_calibration(float _calibration) { calibration = _calibration; }

  void set_count_aggregate(int32_t _caggr) { count_aggregate = _caggr; }

  /**
   * @brief Get Count per Revolution
   */
  int16_t get_cpr(void) { return cpr; }

  float get_calibration(void) { return calibration; }

  float get_calibrated_cpr(void) { return (float)cpr + calibration; }

  float get_revolutions(void) { return (float)get_count_aggregate() / cpr; }

  float get_calibrated_revolutions(void) { return (float)get_count_aggregate() / get_calibrated_cpr(); }
};

#endif // ROBOTLIB_ENCODER_HPP_