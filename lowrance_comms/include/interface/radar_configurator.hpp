#ifndef RADAR_INITIALIZATION_H
#define RADAR_INITIALIZATION_H

#include <string>

// A class for radar initialization
// Each "set" function returns "true" if a given argument is valid

class radar_configurator{
  public:

    // Stores all arguments received via command line
    bool parse_console_args(int argc, char* argv[]);

    // Loads any empty fields with values from the ROS Parameter Server, if they exist
    // If the values exist but are wrong, then they are deleted
    void fillEmptyWithRosParameterServerValues(bool supress_prints = false);

    // Fills empty values with the default ones. Will return false if it is unable
    // to fill the empty values (the card IP doesn't have a default!)
    bool fillEmptyWithDefaults(void);

    // Network card IP methods. Must be an IPv4
    bool setCardIp(std::string network_card_ip);
    bool doesCardIpExist(void);
    std::string getCardIp(void);

    // Radar range methods. Allows ranges from 50 m up to 50000 m
    bool setRange(uint32_t range);
    bool doesRangeExist(void);
    uint32_t getRange(void);

    // Interference Rejection methods. Allows values from 0 (off) to 3 (high)
    bool setInterferenceRejection(uint8_t interference_rejection);
    bool doesInterferenceRejectionExist(void);
    uint8_t getInterferenceRejection(void);

    // Target Boost methods. Allows values 0 (off), 1 (low), 2 (high)
    bool setTargetBoost(uint8_t target_boost);
    bool doesTargetBoostExist(void);
    uint8_t getTargetBoost(void);

    // Local Interference Filter methods. Allows values from 0 (off) to 3 (high)
    bool setLocalIntFilter(uint8_t local_int_filter);
    bool doesLocalIntFilterExist(void);
    uint8_t getLocalIntFilter(void);

    // Scan speed methods. Allows values 0 (normal speed) or 1 (high speed)
    bool setScanSpeed(uint8_t scan_speed);
    bool doesScanSpeedExist(void);
    uint8_t getScanSpeed(void);

    void setAutoGain(bool auto_gain);
    bool doesAutoGainExist(void);
    bool getAutoGain(void);

    bool setManualGainValue(int manual_gain_value);
    bool doesManualGainValueExist(void);
    uint8_t getManualGainValue(void);

    bool setRainFilter(int rain_filter);
    bool doesRainFilterExist(void);
    uint8_t getRainFilter(void);

    void setAutoSeaClutter(bool auto_sea_clutter);
    bool doesAutoSeaClutterExist(void);
    bool getAutoSeaClutter(void);

    bool setSeaClutterFilterValue(int sea_clutter_filter_value);
    bool doesSeaClutterFilterValueExist(void);
    uint8_t getSeaClutterFilterValue(void);

    // Operator overloading for == check
    bool operator== (const radar_configurator& B);
    bool operator!= (const radar_configurator& B);

  private:
    bool network_card_ip_exists = false;
    std::string network_card_ip;

    bool range_exists = false;
    uint32_t range;

    bool interference_rejection_exists = false;
    uint8_t interference_rejection;

    bool target_boost_exists = false;
    uint8_t target_boost;

    bool local_int_filter_exists = false;
    uint8_t local_int_filter;

    bool scan_speed_exists = false;
    uint8_t scan_speed;

    bool auto_gain_exists = false;
    bool auto_gain;

    bool manual_gain_value_exists = false;
    uint8_t manual_gain_value;

    bool rain_filter_exists = false;
    uint8_t rain_filter;

    bool auto_sea_clutter_exists = false;
    bool auto_sea_clutter;

    bool sea_clutter_filter_value_exists = false;
    uint8_t sea_clutter_filter_value;

};

#endif
