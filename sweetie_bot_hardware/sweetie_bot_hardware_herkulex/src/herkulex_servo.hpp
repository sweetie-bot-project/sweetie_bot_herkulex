#ifndef  HERKULEX_SERVO_HPP
#define  HERKULEX_SERVO_HPP

namespace herkulex_servo {

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

			const Register& getByName(const std::string name) const { return name_map.at(name); }
			const Register& getByNum(unsigned int num) const { return registers.at(num); }
			const Register& getByEEPadr(unsigned int eep_adr) const { return eep_map.at(eep_adr); }
			const Register& getByRAMadr(unsigned int ram_adr) const { return ram_map.at(ram_adr); }

			const Register * findByName(const std::string name) const { auto reg = name_map.find(name); return (reg != name_map.end()) ? reg : nullptr; }
			const Register * findByNum(unsigned int num) const { if (num < registers.size()) return &registers[num] else return nullptr; }
			const Register * findByEEPadr(unsigned int eep_adr) const { auto reg = eep_map.find(eep_adr); return (reg != eep_map.end()) ? reg : nullptr; }
			const Register * findByRAMadr(unsigned int ram_adr) const { auto reg = ram_map.find(ram_adr); return (reg != ram_map.end()) ? reg : nullptr; }
	};

	class HerkulexServo
	{
		public: 
			struct PosVel {
				bool error;
				double pos;
				double vel;
			};

			struct State {
				bool error;
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

				enum detail_flag {
					MOVING = 0x01,
					INPOSITION = 0x02,
					INVALID_PACKET_CHECKSUM = 0x04,
					INVALID_PACKET_UNKNOWN_CMD = 0x08,
					INVALID_PACKET_REG_RANGE = 0x10,
					INVALID_PACKET_FRAME_ERROR = 0x20,
					MOTOR_ON = 0x40,
				};
				enum error_flag {
					ERROR_OVER_VOLTAGE = 0x01,
					ERROR_POT_LIMIT = 0x02,
					ERROR_TEMPERATURE = 0x04,
					INVALID_PACKET = 0x08,
					ERROR_OVERLOAD = 0x10,
					ERROR_DRIVER_FAULT = 0x20,
					ERROR_EEP_REGS = 0x40,
					ERROR_MASK = 0x77,
				};
			};

			struct JOGMode {
				unsigned char flags;

				JOGMode() { flags = 0; }
				JOGMode(unsigned char _flags) flags(_flags) {}
				unsigned char() { return flags & JOGMODE_MASK; }

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

		protected:
			std::string name;
			unsigned int hw_id;
			bool reverse;
			int offset;

		public:
			const RegisterMapper& register_mapper;

		protected:
			void reqWrite_impl(HerkulexPacket& req, unsigned int reg_num, unsigned int n, const unsigned int * data) const;
			bool ackRead_impl(const HerkulexPacket& ack, unsigned int reg_num, unsigned int * data, Status& status) const;
			bool ackStatReturn_impl(const HerkulexPacket& ack, Status& status) const;

		public:
			HerkulexServo(const std::string _name&, const RegisterMapper& mapper, unsigned int _hw_id, bool _reverse, int _offset);

			unsigned int getID() { return hw_id; }
			bool isReverse() { return reverse; }
			bool getOffset() { return offset; }

			void reqRead_ram(HerkulexPacket& req, const std::string& reg) const;
			void reqRead_epp(HerkulexPacket& req, string const std::string& reg) const;
			void reqWrite_raw(HerkulexPacket& req, const std::string& reg, unsigned int val) const;
			void reqWrite_epp(HerkulexPacket& req, string const std::string&, unsigned int val) const;
			void reqWriteClearStatus(HerkulexPacket& req) const;
			void reqStat(HerkulexPacket& req) const;
			void reqRollback(HerkulexPacket& req) const;
			void reqReset(HerkulexPacket& req) const;

			void reqIJOGheader(HerkulexPacket& req) const;
			void insertIJOGdata(HerkulexPacket& req, JOGMode mode, unsigned int goal, unsigned int playtime) const;
			void reqSJOGheader(HerkulexPacket& req, unsigned int playtime) const;
			void insertSJOGdata(HerkulexPacket& req, JOGMode mode, unsigned int goal) const;

			bool ackRead_ram(const HerkulexPacket& ack, const std::string& reg, unsigned int& val, Status& status) const;
			bool ackRead_epp(const HerkulexPacket& ack, const std::string& reg, unsigned int& val, Status& status) const;
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
				if (ack.command != HerkulexPacket::ACK_RESET) return false;
				return ackStatReturn_impl(ack, status);
			}

			virtual double convertVelRawToRad(unsigned int raw) const = 0;
			virtual unsigned int convertVelRadToRaw(double vel) const = 0;
			virtual double convertPosRawToRad(unsigned int raw) const = 0;
			virtual unsigned int convertPosRadToRaw(double pos) const = 0;
			virtual double convertTimeRawToSec(unsigned int raw) const = 0;
			virtual unsigned int convertTimeSecToRaw(double pos) const = 0;

			virtual void reqPosVel(HerkulexPacket& req) const = 0;
			virtual bool ackPosVel(HerkulexPacket& ack, double& pos, double& vel, Status& status) = 0;
			virtual void reqState(HerkulexPacket& req) const = 0;
			virtual bool ackState(HerkulexPacket& ack, State& state, Status& status) const = 0;
	};

	class HerkulexServoArray : public std::unordered_map<std::string, std::shared_ptr<HerkulexServo> >
	{
		public: 
			HerkulexServoArray() {}

			void addServo(std::shared_ptr<HerkulexServo> ptr);
			const HerkulexServo& getServo(const std::string name) { return *(this->at(name)); }
	};

};
			
#endif  /*HERKULEX_SERVO_HPP*/
