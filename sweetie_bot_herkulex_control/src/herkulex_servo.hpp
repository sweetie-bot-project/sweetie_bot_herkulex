#ifndef  HERKULEX_SERVO_HPP
#define  HERKULEX_SERVO_HPP

#include <string>
#include <vector>
#include <unordered_map>

#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <sweetie_bot_herkulex_msgs/typekit/HerkulexPacket.h>

namespace herkulex {

namespace servo {

	typedef sweetie_bot_herkulex_msgs::HerkulexPacket HerkulexPacket;

	struct Register {
		signed char reg_num; // acording to manual
		std::string name;
		signed char eep_addr;
		signed char ram_addr;
		unsigned char bytes;
		bool rw;
		std::string description;
	}; 

	typedef std::map<std::string, unsigned int> RegisterValues;

	class RegisterMapper {
		public:
			const std::vector<Register>& registers;
		protected:
			std::unordered_map<std::string, const Register *> name_map;
			std::unordered_map<unsigned int, const Register *> eep_map;
			std::unordered_map<unsigned int, const Register *> ram_map;

		public:
			RegisterMapper(const std::vector<Register>& regs);

			const Register& getByName(const std::string& name) const { 
				auto reg = name_map.find(name); 
				if (reg == name_map.end()) throw std::out_of_range("herkulex::RegisterMapper unknown register \'" + name + "\'.");
				return *(reg->second);
			}
			const Register& getByNum(unsigned int num) const { return registers.at(num); }
			const Register& getByEEPadr(unsigned int eep_adr) const { return *eep_map.at(eep_adr); }
			const Register& getByRAMadr(unsigned int ram_adr) const { return *ram_map.at(ram_adr); }

			const Register * findByName(const std::string& name) const { 
				auto reg = name_map.find(name); 
				return (reg != name_map.end()) ? reg->second : nullptr; 
			}
			const Register * findByNum(unsigned int num) const { 
				if (num < registers.size()) return &registers[num]; 
				else return nullptr; 
			}
			const Register * findByEEPaddr(unsigned int eep_adr) const { 
				auto reg = eep_map.find(eep_adr); 
				return (reg != eep_map.end()) ? reg->second : nullptr; 
			}
			const Register * findByRAMaddr(unsigned int ram_adr) const { 
				auto reg = ram_map.find(ram_adr); 
				return (reg != ram_map.end()) ? reg->second : nullptr; 
			}
	};

	struct PosVel {
		bool error;
		double pos;
		double vel;
	};

	struct RTState {
		double position;
		double velocity;
		double effort;
		unsigned char temperature;
		unsigned char status_error;
	};

	struct State {
		double pos;
		double vel;
		double pwm;
		double pos_goal;
		double pos_desired;
		double vel_desired;
	};

	struct Status {
		unsigned char error;
		unsigned char detail;

		Status() {}
		Status(unsigned int flags) : error(flags & 0xFF), detail((flags >> 8) & 0xFF) {}
		Status(unsigned char _error, unsigned char _detail) : error(_error), detail(_detail) {}
		operator unsigned int() const { return (unsigned int) error + ((unsigned int) detail) << 8; }
		std::string toString() const;
		bool isErrorStatus() const { return error & STATUS_E_ERROR_MASK; }

		enum detail_drs_flag {
			STATUS_D_MOVING = 0x01,
			STATUS_D_INPOSITION = 0x02,
			STATUS_D_INVALID_CHECKSUM = 0x04,
			STATUS_D_UNKNOWN_CMD = 0x08,
			STATUS_D_INVALID_REG_RANGE = 0x10,
			STATUS_D_FRAME_ERROR = 0x20,
		};
		enum detail_sb_flag {
			STATUS_D_RECV_OVERFLOW = 0x04,
			STATUS_D_INVALID_REQ = 0x08,
			STATUS_D_OP_ERROR = 0x20,
			STATUS_D_MOTOR_ON = 0x40,
			STATUS_D_PROTOCOL_ERROR_MASK = STATUS_D_RECV_OVERFLOW | STATUS_D_INVALID_REQ | STATUS_D_INVALID_REG_RANGE | STATUS_D_OP_ERROR,
		};
		enum error_drs_flag {
			STATUS_E_OVER_VOLTAGE = 0x01,
			STATUS_E_POT_LIMIT = 0x02,
			STATUS_E_TEMPERATURE = 0x04,
			STATUS_E_INVALID_PACKET = 0x08,
			STATUS_E_OVERLOAD = 0x10,
			STATUS_E_DRIVER_FAULT = 0x20,
			STATUS_E_EEP_REGS = 0x40,
			STATUS_E_ERROR_MASK = STATUS_E_OVER_VOLTAGE | STATUS_E_POT_LIMIT | STATUS_E_TEMPERATURE | STATUS_E_OVERLOAD | STATUS_E_DRIVER_FAULT | STATUS_E_EEP_REGS,
		};
		enum error_sb_flag {
			STATUS_E_MOTOR_ON = 0x80,
		};
	};

	struct JOGMode {
		unsigned char flags;

		JOGMode() { flags = 0; }
		JOGMode(unsigned char _flags) : flags(_flags) {}
		operator unsigned char() const { return flags & JOGMODE_MASK; }

		enum {
			STOP             = 0x01, // 0b00000001
			POSITION_CONTROL = 0x00, // 0b00000000
			SPEED_CONTROL    = 0x02, // 0b00000010

			LED_OFF          = 0x00, // 0b00000000
			LED_WHITE        = 0x1C, // 0b00011100
			LED_RED          = 0x10, // 0b00010000
			LED_GREEN        = 0x04, // 0b00000100
			LED_BLUE         = 0x08, // 0b00001000
			LED_YELLOW       = 0x14, // 0b00010100
			LED_CYAN         = 0x0C, // 0b00001100
			LED_MAGNETA      = 0x18, // 0b00011000

			JOGMODE_MASK	 = 0x1F,
		};
	};

	class HerkulexServo
	{
		public: 
			typedef  boost::function<bool(const sweetie_bot_herkulex_msgs::HerkulexPacket&)> AckCallback;
			static const unsigned int BROADCAST_ID = 0xfe;

		protected:
			std::string name;
			unsigned int hw_id;
			bool reverse;
			int offset;
			double scale;
			int max_position, min_position; // limits for JOG command

		public:
			const RegisterMapper& register_mapper;

		protected:
			void reqWrite_impl(HerkulexPacket& req, unsigned int reg_num, unsigned int n, const unsigned int * data) const;
			bool ackRead_impl(const HerkulexPacket& ack, unsigned int reg_num, unsigned int * data, Status& status) const;
			bool ackStatReturn_impl(const HerkulexPacket& ack, Status& status) const;

		public:
			HerkulexServo(const std::string& _name, const RegisterMapper& mapper, unsigned int _hw_id, bool _reverse, int _offset, double _scale = 1.0);
			HerkulexServo(const std::string& _name, const RegisterMapper& mapper, unsigned int _hw_id, bool _reverse, int _offset, double _scale, int _min_position, int _max_position);

			// data fields access
			const std::string& getName() const { return name; }
			unsigned int getID() const { return hw_id; }
			bool isReverse() const { return reverse; }
			int getOffset() const { return offset; }
			double getScale() const { return reverse ? -scale : scale; }
			std::pair<int, int> getLimits() { return std::make_pair(min_position, max_position); }

			// Request packets generations.
			void reqRead_ram(HerkulexPacket& req, const std::string& reg) const;
			void reqRead_eep(HerkulexPacket& req, const std::string& reg) const;
			void reqWrite_ram(HerkulexPacket& req, const std::string& reg, unsigned int val) const;
			void reqWrite_eep(HerkulexPacket& req, const std::string&, unsigned int val) const;
			void reqWriteClearStatus(HerkulexPacket& req) const;
			void reqStat(HerkulexPacket& req) const;
			void reqRollback(HerkulexPacket& req) const;
			void reqReset(HerkulexPacket& req) const;
			static void reqResetBroadcast(HerkulexPacket& req); // broadcast version

			// SJOG command generation
			static void reqIJOGheader(HerkulexPacket& req);
			void insertIJOGdata(HerkulexPacket& req, JOGMode mode, unsigned int goal, unsigned int playtime) const;
			virtual void insertIJOGdataConvert(HerkulexPacket& req, JOGMode mode, double goal, double playtime) const;

			// IJOG command generation
			static void reqSJOGheader(HerkulexPacket& req, unsigned int playtime);
			void insertSJOGdata(HerkulexPacket& req, JOGMode mode, unsigned int goal) const;
			virtual void insertSJOGdataConvert(HerkulexPacket& req, JOGMode mode, double goal) const;

			// RT_EXCHANGE command generation
			static void reqRT_EXCHANGEheader(HerkulexPacket& req);
			void insertRT_EXCHANGEdata(HerkulexPacket& req, int position, int velocity, int current) const;
			virtual void insertRT_EXCHANGEdataConvert(HerkulexPacket& req, double position, double velocity, double current) const;

			// Acknowelege packets parse functions.
			bool ackRead_ram(const HerkulexPacket& ack, const std::string& reg, unsigned int& val, Status& status) const;
			bool ackRead_eep(const HerkulexPacket& ack, const std::string& reg, unsigned int& val, Status& status) const;
			bool ackWrite_ram(const HerkulexPacket& ack, Status& status) const {
				if (ack.command != HerkulexPacket::ACK_RAM_WRITE) return false;
				return ackStatReturn_impl(ack, status);
			}
			bool ackWrite_eep(const HerkulexPacket& ack, Status& status) const {
				if (ack.command != HerkulexPacket::ACK_EEP_WRITE) return false;
				return ackStatReturn_impl(ack, status);
			}
			bool ackWriteClearStatus(const HerkulexPacket& ack, Status& status) const {
				if (ack.command != HerkulexPacket::ACK_RAM_WRITE) return false;
				return ackStatReturn_impl(ack, status);
			}
			bool ackStat(const HerkulexPacket& ack, Status& status) const {
				if (ack.command != HerkulexPacket::ACK_STAT) return false;
				return ackStatReturn_impl(ack, status);
			}
			bool ackRollback(const HerkulexPacket& ack, Status& status) const {
				if (ack.command != HerkulexPacket::ACK_ROLLBACK) return false;
				return ackStatReturn_impl(ack, status);
			}
			bool ackReset(const HerkulexPacket& ack, Status& status) const {
				if (ack.command != HerkulexPacket::ACK_REBOOT) return false;
				return ackStatReturn_impl(ack, status);
			}

			// RT_EXCHANGE acknowlege packet parse
			virtual bool ackRT_EXCHANGE(const HerkulexPacket& ack, RTState& state) const;

			// Acknowlege packets parse callbacks. Callback can be stored in AckCallback variable and passed to function to perform
			// multiple parse packets attemts.
			AckCallback ackCallbackRead_ram(const std::string& reg, unsigned int& val, Status& status) const {
				return boost::bind(&HerkulexServo::ackRead_ram, this, _1, boost::cref(reg), boost::ref(val), boost::ref(status));
			}
			AckCallback ackCallbackRead_eep(const std::string& reg, unsigned int& val, Status& status) const {
				return boost::bind(&HerkulexServo::ackRead_eep, this, _1, boost::cref(reg), boost::ref(val), boost::ref(status));
			}
			AckCallback ackCallbackWrite_ram(Status& status) const {
				return boost::bind(&HerkulexServo::ackWrite_ram, this, _1, boost::ref(status));
			}
			AckCallback ackCallbackWrite_eep(Status& status) const {
				return boost::bind(&HerkulexServo::ackWrite_eep, this, _1, boost::ref(status));
			}
			AckCallback ackCallbackWriteClearStatus(Status& status) const {
				return boost::bind(&HerkulexServo::ackWriteClearStatus, this, _1, boost::ref(status));
			}
			AckCallback ackCallbackStat(Status& status) const {
				return boost::bind(&HerkulexServo::ackStat, this, _1, boost::ref(status));
			}
			AckCallback ackCallbackRollback(Status& status) const {
				return boost::bind(&HerkulexServo::ackRollback, this, _1, boost::ref(status));
			}
			AckCallback ackCallbackReset(Status& status) const {
				return boost::bind(&HerkulexServo::ackReset, this, _1, boost::ref(status));
			}

			AckCallback ackCallbackRT_EXCHANGE(RTState& state) const {
				return boost::bind(&HerkulexServo::ackRT_EXCHANGE, this, _1, boost::ref(state));
			}

			virtual double convertPosRawToRad(unsigned int raw) const = 0;
			virtual unsigned int convertPosRadToRaw(double pos) const = 0;
			virtual double convertVelRawToRad(unsigned int raw) const = 0;
			virtual unsigned int convertVelRadToRaw(double vel) const = 0;
			virtual double convertTimeRawToSec(unsigned int raw) const = 0;
			virtual unsigned int convertTimeSecToRaw(double pos) const = 0;
			virtual double convertEffortRawToHm(unsigned int raw) const = 0;
			virtual unsigned int convertEffortHmToRaw(double effort) const = 0;
			virtual double convertVoltageRawToVolts(unsigned int raw) const = 0;
			virtual double convertTemperatureRawToCelsius(unsigned int raw) const = 0;

			virtual void reqPosVel(HerkulexPacket& req) const = 0;
			virtual bool ackPosVel(const HerkulexPacket& ack, double& pos, double& vel, Status& status) const = 0;
			AckCallback ackCallbackPosVel(double& pos, double& vel, Status& status) const {
				return boost::bind(&HerkulexServo::ackPosVel, this, _1, boost::ref(pos), boost::ref(vel), boost::ref(status));
			}

			virtual void reqPosVelExtended(HerkulexPacket& req) const = 0;
			virtual bool ackPosVelExtended(const HerkulexPacket& ack, State& state, Status& status) const = 0;
			AckCallback ackCallbackackPosVelExtended(State& state, Status& status) const {
				return boost::bind(&HerkulexServo::ackPosVelExtended, this, _1, boost::ref(state), boost::ref(status));
			}
	};


	typedef std::shared_ptr<HerkulexServo> HerkulexServoSharedPtr;
	typedef std::unordered_map< std::string, HerkulexServoSharedPtr > MapNameToHerkulexServo;
	typedef std::unordered_map< unsigned char, HerkulexServoSharedPtr > MapHwIdToHerkulexServo;

} // namespace servo

} // namespace herkulex
			
#endif  /*HERKULEX_SERVO_HPP*/
