#ifndef  HERKULEX_PROTOCOL_HPP
#define  HERKULEX_PROTOCOL_HPP

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
			RegisterMapper(const std::vector<Register>& regs);


#endif  /*HERKULEX_PROTOCOL_HPP*/
