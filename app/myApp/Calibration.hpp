#ifndef CH_NTB_EEDURO_DEMO_CALIBRATION_HPP_
#define CH_NTB_EEDURO_DEMO_CALIBRATION_HPP_

#include <eeros/config/FileConfig.hpp>

namespace eeduro {
	namespace delta {
		class Calibration : public eeros::config::FileConfig {
		public:
			Calibration();
			Calibration(const char *path);
			virtual ~Calibration();
			
			virtual void loadDefaults();
			
			int getBlock(int pos, double z);
			
			struct {
				double x;
				double y;
				double r;
				double level12;
				double level23;
				double level30;
				double zblock[4];
			} position[4];
			double transportation_height;
		};
	}
}

#endif // CH_NTB_EEDURO_DEMO_CALIBRATION_HPP_
