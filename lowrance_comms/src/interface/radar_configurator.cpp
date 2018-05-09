#include "interface/radar_configurator.hpp"

#include "ros/ros.h"

// Auxiliary function. Checks if the next argument both exists and is not a command
static bool next_argument_is_valid(int argc, char* argv[], int current_arg){
  if((current_arg+1) >= argc){
    return false;
  }
  if(argv[current_arg+1][0] == '-'){
    return false;
  }
  return true;
}

// Stores all arguments received via command line
bool radar_configurator::parse_console_args(int argc, char* argv[]){

  for(auto i=0; i != argc; i++){
    if( (argv[i][0] == '-') && (strlen(argv[i]) == 2) ){

      switch(argv[i][1]){
        case 'i':
          if(next_argument_is_valid(argc, argv, i) == false){
            ROS_ERROR("No IP was provided to argument -i");
            return false;
          }
          if(this->setCardIp(std::string(argv[i+1])) == false){
            ROS_ERROR("An invalid IP was provided");
            return false;
          }
          break;

        case 'r':
          if(next_argument_is_valid(argc, argv, i) == false){
            ROS_ERROR("No range was provided to argument -r");
            return false;
          }
          try{
            if(this->setRange(std::stoul(std::string(argv[i+1]))) == false){
              ROS_ERROR("An invalid radar range for argument -r was provided. Valid ranges: [50;24000] meters");
              return false;
            }
          }
          catch(...){
            ROS_ERROR("Couldn't resolve range value \"%s\" for argument -r",argv[i+1]);
            return false;
          }
          break;

        case 't':
          if(next_argument_is_valid(argc, argv, i) == false){
            ROS_ERROR("No value was provided to argument -t");
            return false;
          }
          try{
            if(this->setInterferenceRejection(std::stoul(std::string(argv[i+1]))) == false){
              ROS_ERROR("An invalid interference rejection value for argument -t was provided. Valid options: 0 (off) to 3 (high)");
              return false;
            }
          }
          catch(...){
            ROS_ERROR("Couldn't resolve interference rejection value \"%s\" for argument -t",argv[i+1]);
            return false;
          }
          break;

        case 'b':
          if(next_argument_is_valid(argc, argv, i) == false){
            ROS_ERROR("No value was provided to argument -b");
            return false;
          }
          try{
            if(this->setTargetBoost(std::stoul(std::string(argv[i+1]))) == false){
              ROS_ERROR("An invalid target boost value for argument -b was provided. Valid options: 0 (off) to 2 (high)");
              return false;
            }
          }
          catch(...){
            ROS_ERROR("Couldn't resolve target boost value \"%s\" for argument -b",argv[i+1]);
            return false;
          }
          break;

        case 'l':
          if(next_argument_is_valid(argc, argv, i) == false){
            ROS_ERROR("No value was provided to argument -l");
            return false;
          }
          try{
            if(this->setLocalIntFilter(std::stoul(std::string(argv[i+1]))) == false){
              ROS_ERROR("An invalid local interference filter value for argument -l was provided. Valid options: 0 (off) to 3 (high)");
              return false;
            }
          }
          catch(...){
            ROS_ERROR("Couldn't resolve local interference filter value \"%s\" for argument -l",argv[i+1]);
            return false;
          }
          break;

        case 's':
          if(next_argument_is_valid(argc, argv, i) == false){
            ROS_ERROR("No value was provided to argument -s");
            return false;
          }
          try{
            if(this->setScanSpeed(std::stoul(std::string(argv[i+1]))) == false){
              ROS_ERROR("An invalid scan speed value for argument -s was provided. Valid options: 0 (normal) or 1 (fast)");
              return false;
            }
          }
          catch(...){
            ROS_ERROR("Couldn't resolve scan speed value \"%s\" for argument -l",argv[i+1]);
            return false;
          }
          break;

        case 'a':
          if(next_argument_is_valid(argc, argv, i) == false){
            ROS_ERROR("No value was provided to argument -a");
            return false;
          }
          int temp_auto_gain;
          try{
            temp_auto_gain = std::stoul(std::string(argv[i+1]));
            if(temp_auto_gain == 1){
              setAutoGain(true);
            }else if(temp_auto_gain == 0){
              setAutoGain(false);
            }else{
              ROS_ERROR("An invalid auto gain value for argument -a was provided. Valid options: 0 (manual) or 1 (auto)");
              return false;
            }
          }
          catch(...){
            ROS_ERROR("Couldn't resolve auto gain value \"%s\" for argument -a",argv[i+1]);
            return false;
          }
          break;
        case 'g':
          if(next_argument_is_valid(argc, argv, i) == false){
            ROS_ERROR("No value was provided to argument -g");
            return false;
          }
          try{
            if(this->setManualGainValue(std::stoul(std::string(argv[i+1]))) == false){
              ROS_ERROR("An invalid manual gain value for argument -g was provided. Valid manual gain values: [0;255]");
              return false;
            }
          }
          catch(...){
            ROS_ERROR("Couldn't resolve manual gain value \"%s\" for argument -g",argv[i+1]);
            return false;
          }
          break;
        case 'f':
          if(next_argument_is_valid(argc, argv, i) == false){
            ROS_ERROR("No value was provided to argument -f");
            return false;
          }
          try{
            if(this->setRainFilter(std::stoul(std::string(argv[i+1]))) == false){
              ROS_ERROR("An invalid rain filter value for argument -f was provided. Valid rain filter values: [1;80]");
              return false;
            }
          }
          catch(...){
            ROS_ERROR("Couldn't resolve rain filter value \"%s\" for argument -f",argv[i+1]);
            return false;
          }
          break;
        case 'e':
          if(next_argument_is_valid(argc, argv, i) == false){
            ROS_ERROR("No value was provided to argument -e");
            return false;
          }
          int temp_auto_sea_clutter;
          try{
            temp_auto_sea_clutter = std::stoul(std::string(argv[i+1]));
            if(temp_auto_sea_clutter == 1){
              setAutoSeaClutter(true);
            }else if(temp_auto_sea_clutter == 0){
              setAutoSeaClutter(false);
            }else{
              ROS_ERROR("An invalid auto sea clutter value for argument -e was provided. Valid options: 0 (manual) or 1 (auto)");
              return false;
            }
          }
          catch(...){
            ROS_ERROR("Couldn't resolve auto sea clutter value \"%s\" for argument -e",argv[i+1]);
            return false;
          }
          break;
        case 'u':
          if(next_argument_is_valid(argc, argv, i) == false){
            ROS_ERROR("No value was provided to argument -u");
            return false;
          }
          try{
            if(this->setSeaClutterFilterValue(std::stoul(std::string(argv[i+1]))) == false){
              ROS_ERROR("An invalid sea clutter filter value for argument -u was provided. Valid sea clutter filter values: [0;255]");
              return false;
            }
          }
          catch(...){
            ROS_ERROR("Couldn't resolve sea clutter filter value \"%s\" for argument -u",argv[i+1]);
            return false;
          }
          break;
        default:
          break;
      }
    }
  }
  return true;
}

// Loads any empty fields with values from the ROS Parameter Server, if they exist
// If the values exist but are wrong, then they are deleted
void radar_configurator::fillEmptyWithRosParameterServerValues(bool supress_prints){

  if(this->doesCardIpExist() == true){
    ros::param::set("/lowrance_comms/network_card_ip",this->getCardIp());
    if(!supress_prints) ROS_INFO("Updating card IP in ROS Parameter Server");
  }else{
    std::string network_card_ip;
    if(ros::param::get("/lowrance_comms/network_card_ip",network_card_ip) == true){
      if(this->setCardIp(network_card_ip) == true){
        if(!supress_prints) ROS_INFO("Loaded card IP from ROS Parameter Server");
      }else{
        ROS_WARN("Invalid card IP stored in ROS Parameter Server. Deleting...");
        ros::param::del("/lowrance_comms/network_card_ip");
      }
    }
  }

  if(this->doesRangeExist() == true){
    ros::param::set("/lowrance_comms/range",(int)this->getRange());
    if(!supress_prints) ROS_INFO("Updating range in ROS Parameter Server");
  }else{
    int range;
    if(ros::param::get("/lowrance_comms/range", range) == true){
      if(this->setRange((uint32_t)range) == true){
        if(!supress_prints) ROS_INFO("Loaded range from ROS Parameter Server");
      }else{
        ROS_WARN("Invalid range stored in ROS Paremeter Server. Deleting...");
        ros::param::del("/lowrance_comms/range");
      }
    }
  }

  if(this->doesInterferenceRejectionExist() == true){
    ros::param::set("/lowrance_comms/interference_rejection",(int)this->getInterferenceRejection());
    if(!supress_prints) ROS_INFO("Updating interference rejection value in ROS Parameter Server");
  }else{
    int interference_rejection;
    if(ros::param::get("/lowrance_comms/interference_rejection", interference_rejection) == true){
      if(this->setInterferenceRejection((uint8_t)interference_rejection) == true){
        if(!supress_prints) ROS_INFO("Loaded interference rejection value from ROS Parameter Server");
      }else{
        ROS_WARN("Invalid interference rejection value stored in ROS Paremeter Server. Deleting...");
        ros::param::del("/lowrance_comms/interference_rejection");
      }
    }
  }

  if(this->doesTargetBoostExist() == true){
    ros::param::set("/lowrance_comms/target_boost",(int)this->getTargetBoost());
    if(!supress_prints) ROS_INFO("Updating target boost value in ROS Parameter Server");
  }else{
    int target_boost;
    if(ros::param::get("/lowrance_comms/target_boost",target_boost) == true){
      if(this->setTargetBoost((uint8_t)target_boost) == true){
        if(!supress_prints) ROS_INFO("Loaded target boost value from ROS Parameter Server");
      }else{
        ROS_WARN("Invalid target boost value stored in ROS Paremeter Server. Deleting...");
        ros::param::del("/lowrance_comms/target_boost");
      }
    }
  }

  if(this->doesLocalIntFilterExist() == true){
    ros::param::set("/lowrance_comms/local_int_filter",(int)this->getLocalIntFilter());
    if(!supress_prints) ROS_INFO("Updating local interference filter value in ROS Parameter Server");
  }else{
    int local_int_filter;
    if(ros::param::get("/lowrance_comms/local_int_filter",local_int_filter) == true){
      if(this->setLocalIntFilter((uint8_t)local_int_filter) == true){
        if(!supress_prints) ROS_INFO("Loaded local interference filter value from ROS Parameter Server");
      }else{
        ROS_WARN("Invalid local interference filter value stored in ROS Paremeter Server. Deleting...");
        ros::param::del("/lowrance_comms/local_int_filter");
      }
    }
  }

  if(this->doesScanSpeedExist() == true){
    ros::param::set("/lowrance_comms/scan_speed", (int)this->getScanSpeed());
    if(!supress_prints) ROS_INFO("Updating scan speed value in ROS Parameter Server");
  }else{
    int scan_speed;
    if(ros::param::get("/lowrance_comms/scan_speed", scan_speed) == true){
      if(this->setScanSpeed((uint8_t)scan_speed) == true){
        if(!supress_prints) ROS_INFO("Loaded scan speed value from ROS Parameter Server");
      }else{
        ROS_WARN("Invalid scan speed value stored in ROS Paremeter Server. Deleting...");
        ros::param::del("/lowrance_comms/scan_speed");
      }
    }
  }

  if(this->doesAutoGainExist() == true){
    ros::param::set("/lowrance_comms/auto_gain", this->getAutoGain());
    if(!supress_prints) ROS_INFO("Updating auto gain option in ROS Parameter Server");
  }else{
    bool auto_gain;
    if(ros::param::get("/lowrance_comms/auto_gain", auto_gain) == true){
      this->setAutoGain(auto_gain);
      if(!supress_prints) ROS_INFO("Loaded auto gain option from ROS Parameter Server");
    }
  }

  if(this->doesManualGainValueExist() == true){
    ros::param::set("/lowrance_comms/manual_gain_value", this->getManualGainValue());
    if(!supress_prints) ROS_INFO("Updating manual gain value in ROS Parameter Server");
  }else{
    int manual_gain_value;
    if(ros::param::get("/lowrance_comms/manual_gain_value", manual_gain_value) == true){
      if(this->setManualGainValue(manual_gain_value) == true){
        if(!supress_prints) ROS_INFO("Loaded manual gain value from ROS Parameter Server");
      }else{
        ROS_WARN("Invalid manual gain value stored in ROS Paremeter Server. Deleting...");
        ros::param::del("/lowrance_comms/manual_gain_value");
      }
    }
  }

  if(this->doesRainFilterExist() == true){
    ros::param::set("/lowrance_comms/rain_filter", this->getRainFilter());
    if(!supress_prints) ROS_INFO("Updating rain filter value in ROS Parameter Server");
  }else{
    int rain_filter;
    if(ros::param::get("/lowrance_comms/rain_filter", rain_filter) == true){
      if(this->setRainFilter(rain_filter) == true){
        if(!supress_prints) ROS_INFO("Loaded rain filter value from ROS Parameter Server");
      }else{
        ROS_WARN("Invalid rain filter value stored in ROS Paremeter Server. Deleting...");
        ros::param::del("/lowrance_comms/rain_filter");
      }
    }
  }

  if(this->doesAutoSeaClutterExist() == true){
    ros::param::set("/lowrance_comms/auto_sea_clutter", this->getAutoSeaClutter());
    if(!supress_prints) ROS_INFO("Updating auto sea clutter option in ROS Parameter Server");
  }else{
    bool auto_sea_clutter;
    if(ros::param::get("/lowrance_comms/auto_sea_clutter", auto_sea_clutter) == true){
      this->setAutoSeaClutter(auto_sea_clutter);
      if(!supress_prints) ROS_INFO("Loaded auto sea clutter option from ROS Parameter Server");
    }
  }

  if(this->doesSeaClutterFilterValueExist() == true){
    ros::param::set("/lowrance_comms/sea_clutter_filter_value",(int)this->getSeaClutterFilterValue());
    if(!supress_prints) ROS_INFO("Updating sea clutter filter value in ROS Parameter Server");
  }else{
    int sea_clutter_filter_value;
    if(ros::param::get("/lowrance_comms/sea_clutter_filter_value", sea_clutter_filter_value) == true){
      if(this->setSeaClutterFilterValue(sea_clutter_filter_value) == true){
        if(!supress_prints) ROS_INFO("Loaded sea clutter filter value from ROS Parameter Server");
      }else{
        ROS_WARN("Invalid sea clutter filter value stored in ROS Paremeter Server. Deleting...");
        ros::param::del("/lowrance_comms/sea_clutter_filter_value");
      }
    }
  }
}


bool radar_configurator::fillEmptyWithDefaults(void){
  if(this->doesCardIpExist() == false){
    ROS_ERROR("Network card IP not found. Please provide an IP via command line with the argument -i");
    return false;
  }

  if(this->doesRangeExist() == false){
    ROS_WARN("Loading default range setting (6km)");
    this->setRange(6000);
    ros::param::set("/lowrance_comms/range", 6000);
  }

  if(this->doesInterferenceRejectionExist() == false){
    ROS_WARN("Loading default interference rejection value (0 = off)");
    this->setInterferenceRejection(0);
    ros::param::set("/lowrance_comms/interference_rejection", 0);
  }

  if(this->doesTargetBoostExist() == false){
    ROS_WARN("Loading default target boost value (0 = off)");
    this->setTargetBoost(0);
    ros::param::set("/lowrance_comms/target_boost", 0);
  }

  if(this->doesLocalIntFilterExist() == false){
    ROS_WARN("Loading default local interference filter value (0 = off)");
    this->setLocalIntFilter(0);
    ros::param::set("/lowrance_comms/local_int_filter", 0);
  }

  if(this->doesScanSpeedExist() == false){
    ROS_WARN("Loading default scan speed value (0 = normal)");
    this->setScanSpeed(0);
    ros::param::set("/lowrance_comms/scan_speed", 0);
  }

  if(this->doesAutoGainExist() == false){
    ROS_WARN("Loading default auto gain value (true)");
    this->setAutoGain(true);
    ros::param::set("/lowrance_comms/auto_gain", true);
  }

  if(this->doesManualGainValueExist() == false){
    if(this->getAutoGain() == true){
      ROS_INFO("Loading default manual gain value (128). Note: no effect with auto gain enabled");
    }else{
      ROS_WARN("Loading default manual gain value (128)");
    }
    this->setManualGainValue(128);
    ros::param::set("/lowrance_comms/manual_gain_value", 128);
  }

  if(this->doesRainFilterExist() == false){
    ROS_WARN("Loading default rain filter value (77)");
    this->setRainFilter(77);
    ros::param::set("/lowrance_comms/rain_filter", 77);
  }

  if(this->doesAutoSeaClutterExist() == false){
    ROS_WARN("Loading default auto sea clutter value (true)");
    this->setAutoSeaClutter(true);
    ros::param::set("/lowrance_comms/auto_sea_clutter", true);
  }

  if(this->doesSeaClutterFilterValueExist() == false){
    if(this->getAutoSeaClutter() == true){
      ROS_INFO("Loading default sea clutter filter value (128). Note: no effect with auto sea clutter enabled");
    }else{
      ROS_WARN("Loading default sea clutter filter value (128)");
    }
    this->setSeaClutterFilterValue(128);
    ros::param::set("/lowrance_comms/sea_clutter_filter_value", 128);
  }

  return true;
}

bool radar_configurator::operator== (const radar_configurator& B){
  if (this->network_card_ip_exists != B.network_card_ip_exists){
    return false;
  }

  if (this->network_card_ip != B.network_card_ip){
    return false;
  }

  if (this->range_exists != B.range_exists){
    return false;
  }

  if (this->range != B.range){
    return false;
  }

  if (this->interference_rejection_exists != B.interference_rejection_exists){
    return false;
  }

  if (this->interference_rejection != B.interference_rejection){
    return false;
  }

  if (this->target_boost_exists != B.target_boost_exists){
    return false;
  }

  if (this->target_boost != B.target_boost){
    return false;
  }

  if (this->local_int_filter_exists != B.local_int_filter_exists){
    return false;
  }

  if (this->local_int_filter != B.local_int_filter){
    return false;
  }

  if (this->scan_speed_exists != B.scan_speed_exists){
    return false;
  }

  if (this->scan_speed != B.scan_speed){
    return false;
  }

  if (this->auto_gain_exists != B.auto_gain_exists){
    return false;
  }

  if (this->auto_gain != B.auto_gain){
    return false;
  }

  if(this->manual_gain_value_exists != B.manual_gain_value_exists){
    return false;
  }

  if(this->manual_gain_value != B.manual_gain_value){
    return false;
  }

  if(this->rain_filter_exists != B.rain_filter_exists){
    return false;
  }

  if(this->rain_filter != B.rain_filter){
    return false;
  }

  if(this->auto_sea_clutter_exists != B.auto_sea_clutter_exists){
    return false;
  }

  if(this->auto_sea_clutter != B.auto_sea_clutter){
    return false;
  }

  if(this->auto_sea_clutter_exists != B.auto_sea_clutter_exists){
    return false;
  }

  if(this->sea_clutter_filter_value != B.sea_clutter_filter_value){
    return false;
  }

  return true;
}

bool radar_configurator::operator!= (const radar_configurator& B){
  return !((*this)==B);
}

// radar_init_list get and set methods
bool radar_configurator::setCardIp(std::string network_card_ip){

  auto current = 0;
  auto next = -1;
  auto field_counter = 0;

  do{
    current = next + 1;
    next = network_card_ip.find_first_of(".", current );

    if(field_counter ==4){
      return false;
    }

    field_counter++;
    unsigned long field;
    try{
      field = std::stoul(network_card_ip.substr( current, next - current ));
    }
    catch(...){
      return false;
    }

    if(field > 255 || field < 0){
      return false;
    }

  }while (next != std::string::npos);

  this->network_card_ip = network_card_ip;
  this->network_card_ip_exists = true;
  return true;
}

bool radar_configurator::doesCardIpExist(void){
  return this->network_card_ip_exists;
}

std::string radar_configurator::getCardIp(void){
  if(this->network_card_ip_exists == true){
    return this->network_card_ip;
  }else{
    return "";
  }
}

bool radar_configurator::setRange(uint32_t range){
  if(range>=50 && range<=50000){
    this->range = range;
    this->range_exists = true;
    return true;
  }else{
    return false;
  }
}

bool radar_configurator::doesRangeExist(void){
  return this->range_exists;
}

uint32_t radar_configurator::getRange(void){
  if(this->range_exists == true){
    return this->range;
  }else{
    return 0;
  }
}

bool radar_configurator::setInterferenceRejection(uint8_t interference_rejection){
  if(interference_rejection>=0 &&  interference_rejection<=3){
    this->interference_rejection = interference_rejection;
    this->interference_rejection_exists = true;
    return true;
  }else{
    return false;
  }
}

bool radar_configurator::doesInterferenceRejectionExist(void){
  return interference_rejection_exists;
}

uint8_t radar_configurator::getInterferenceRejection(void){
  if(this->interference_rejection_exists == true){
    return this->interference_rejection;
  }else{
    return 0;
  }
}

bool radar_configurator::setTargetBoost(uint8_t target_boost){
  if(target_boost>=0 && target_boost<=2){
    this->target_boost = target_boost;
    this->target_boost_exists = true;
    return true;
  }else{
    return false;
  }
}

bool radar_configurator::doesTargetBoostExist(void){
  return this->target_boost_exists;
}

uint8_t radar_configurator::getTargetBoost(void){
  if(this->target_boost_exists == true){
    return this->target_boost;
  }else{
    return 0;
  }
}

bool radar_configurator::setLocalIntFilter(uint8_t local_int_filter){
  if(local_int_filter>=0 &&  local_int_filter<=3){
    this->local_int_filter = local_int_filter;
    this->local_int_filter_exists = true;
    return true;
  }else{
    return false;
  }
}

bool radar_configurator::doesLocalIntFilterExist(void){
  return local_int_filter_exists;
}

uint8_t radar_configurator::getLocalIntFilter(void){
  if(this->local_int_filter_exists == true){
    return this->local_int_filter;
  }else{
    return 0;
  }
}

bool radar_configurator::setScanSpeed(uint8_t scan_speed){
  if(scan_speed == 0 || scan_speed == 1){
    this->scan_speed = scan_speed;
    this->scan_speed_exists = true;
    return true;
  }else{
    return false;
  }
}

bool radar_configurator::doesScanSpeedExist(void){
  return this->scan_speed_exists;
}

uint8_t radar_configurator::getScanSpeed(void){
  if(this->scan_speed_exists == true){
    return this->scan_speed;
  }else{
    return 0;
  }
}

void radar_configurator::setAutoGain(bool auto_gain){
  this->auto_gain = auto_gain;
  this->auto_gain_exists = true;
}

bool radar_configurator::doesAutoGainExist(void){
  return this->auto_gain_exists;
}

bool radar_configurator::getAutoGain(void){
  return this->auto_gain;
}

bool radar_configurator::setManualGainValue(int manual_gain_value){
  if(manual_gain_value>=0 && manual_gain_value<=255){
    this->manual_gain_value = (uint8_t) manual_gain_value;
    this->manual_gain_value_exists = true;
    return true;
  }else{
    return false;
  }
}

bool radar_configurator::doesManualGainValueExist(void){
  return this->manual_gain_value_exists;
}

uint8_t radar_configurator::getManualGainValue(void){
  if(this->manual_gain_value_exists == true){
    return this->manual_gain_value;
  }else{
    return 128;
  }
}

bool radar_configurator::setRainFilter(int rain_filter){
  if(rain_filter>=1 && rain_filter<=80){
    this->rain_filter = (uint8_t) rain_filter;
    this->rain_filter_exists = true;
    return true;
  }else{
    return false;
  }
}

bool radar_configurator::doesRainFilterExist(void){
  return this->rain_filter_exists;
}

uint8_t radar_configurator::getRainFilter(void){
  if(this->rain_filter_exists == true){
    return this->rain_filter;
  }else{
    return 77;
  }
}

void radar_configurator::setAutoSeaClutter(bool auto_sea_clutter){
  this->auto_sea_clutter = auto_sea_clutter;
  this->auto_sea_clutter_exists = true;
}

bool radar_configurator::doesAutoSeaClutterExist(void){
  return this->auto_sea_clutter_exists;
}

bool radar_configurator::getAutoSeaClutter(void){
  return this->auto_sea_clutter;
}

bool radar_configurator::setSeaClutterFilterValue(int sea_clutter_filter_value){
  if(sea_clutter_filter_value>=0 && sea_clutter_filter_value<=255){
    this->sea_clutter_filter_value = (uint8_t) sea_clutter_filter_value;
    this->sea_clutter_filter_value_exists = true;
    return true;
  }else{
    return false;
  }
}

bool radar_configurator::doesSeaClutterFilterValueExist(void){
  return this->sea_clutter_filter_value_exists;
}

uint8_t radar_configurator::getSeaClutterFilterValue(void){
  if(this->sea_clutter_filter_value_exists == true){
    return this->sea_clutter_filter_value;
  }else{
    return 128;
  }
}
