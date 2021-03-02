//! @file FRAgenerator2.hh
//! @brief FRA用信号生成器
//!
//! Frequency Response Analysis のための信号生成器
//! 静止摩擦の影響を考慮してできるようにした。
//!
//! @date 2021/02/10
//! @author Yuki YOKOKURA & Muto Hirotaka & Kenjirou Oomori
//
// Copyright (C) 2011-2019 Yuki YOKOKURA
// This program is free software;
// you can redistribute it and/or modify it under the terms of the FreeBSD License.
// For details, see the License.txt file.

#ifndef FRAGENERATOR2
#define FRAGENERATOR2

#include <tuple>

namespace ARCS {	// ARCS名前空間
	//! @brief FRA用信号生成器
	class FRAgenerator2 {
		public:
			FRAgenerator2(
				const double FreqMin,		// [Hz]開始周波数
				const double FreqMax,	 	// [Hz]終了周波数
				const double FreqStep,		// [Hz]周波数ステップ
				const double NumIntg,		// [-] 積分周期 (1周波数につき何回sin波を入力するか)
				const double TimeSta		// [s] FRA開始時刻
			);				//!< コンストラクタ
			~FRAgenerator2();//!< デストラクタ
			std::tuple<double, double> GetSignal(double Au, double Bu, double t);	//!< FRA信号出力関数
			
		private:
			FRAgenerator2(const FRAgenerator2&) = delete;					//!< コピーコンストラクタ使用禁止
			const FRAgenerator2& operator=(const FRAgenerator2&) = delete;//!< 代入演算子使用禁止
			const double fmin;	//!< [Hz]開始周波数
			const double fmax;	//!< [Hz]終了周波数
			const double fstep;	//!< [Hz]周波数ステップ
			const double Ni;	//!< [-] 積分周期  (1周波数につき何回sin波を入力するか)
			const double Tsta;	//!< [s] FRA開始時刻
			bool isEnd;			//!< FRA終了フラグ
			double f = fmin;	//!< [Hz]現在の周波数	
			double tini;		//!< [s] 各周波数ごとの開始時刻
	};
}

#endif

