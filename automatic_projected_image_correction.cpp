#include <fstream>
#include <iostream>
#include <cmath>
#include <stdio.h>
#include <string>
#include <vector>
#include "json.hpp"

#include <opencv2\opencv.hpp>
#ifdef _DEBUG
#pragma comment(lib, "opencv_world4110d.lib")
#else
#pragma comment(lib, "opencv_world4110.lib")
#endif

using namespace std;
using json = nlohmann::json;

//arMarkerの情報
#define ARMARKER_SIZE 8.1		//arMarkerの大きさ[cm]
#define ARMARKER_NUM 8			//作成したarMarkerの枚数

//vdの情報
#define VD_ARMARKER_POS_X 6.0	//vdに張り付けたarMarkerの位置x
#define VD_ARMARKER_POS_Y 6.0	//vdに張り付けたarMarkerの位置y
#define VD_ARMARKER_POS_Z -2.2	//vdに張り付けたarMarkerの位置z(下が正)
#define VD_HEIGHTS 10.0			//vdの天板間の距離[cm]
#define VD_ARMARKER_ID 1		//vdに貼り付けたarMarkerのID

//webカメラの解像度
#define WEBCAM_COLS 2592
#define WEBCAM_ROWS 1944
#define CAMERA_ID 0			//カメラオブジェクトを作成するときの引数
#define IMG_PATH "./testImgs03/testImg"	//webCam画像を保存するパス
#define IMG_TYPE ".bmp"			//webCam画像の型

//レーザープロジェクターの情報
#define PROJ_IMG_WIDTH 1280		//プロジェクターの横画素数
#define PROJ_IMG_HEIGHT 720		//プロジェクターの縦画素数
#define AOV_X 0.680678			//39°
#define PROJ_ARMARKER_POS_X 0.0	//projに張り付けたarMarkerの位置x
#define PROJ_ARMARKER_POS_Y 0.0	//projに張り付けたarMarkerの位置y
#define PROJ_ARMARKER_POS_Z 0.0 /*-3.55*/ //projに張り付けたarMarkerの位置z
#define PROJ_ARMARKER_ID 0		//プロジェクターに貼り付けたarMarkerのID

//ArUcoマーカーの位置関係，理想値
#define DIST_X_PROJ_VD 0.0
#define DIST_Y_PROJ_VD 0.0
#define DIST_Z_PROJ_VD 24.05 //16.45
#define DEG_ROLL 0.0
#define DEG_PTICH 0.0
#define DEG_YAW 0.0

//スライダーの設定
#define SLIDER_SCALE 100.0	//スライダーの1目盛りが 1/SLIDER_SCALE


//-----元の糸座標---------------------------------------------------
//糸座標を保存する関数、64本
vector<cv::Point3d> setThreadsCoordinate64() {
	vector<cv::Point3d> threads_coordinate;

	//糸64本の場合
	//8~
	threads_coordinate.push_back({ 3.5, 6.7, 0.0 });
	threads_coordinate.push_back({ 3.2, 3.5, 0.0 });
	threads_coordinate.push_back({ 2.8, 4.1, 0.0 });
	threads_coordinate.push_back({ 3.3, 7.1, 0.0 });
	threads_coordinate.push_back({ 3.1, 4.4, 0.0 });

	//13~
	threads_coordinate.push_back({ 3.4, 7.8, 0.0 });
	threads_coordinate.push_back({ 3.0, 8.4, 0.0 });
	threads_coordinate.push_back({ 2.9, 5.8, 0.0 });
	threads_coordinate.push_back({ 4.1, 7.7, 0.0 });
	threads_coordinate.push_back({ 3.8, 6.0, 0.0 });

	//18~
	threads_coordinate.push_back({ 4.3, 3.2, 0.0 });
	threads_coordinate.push_back({ 4.0, 3.8, 0.0 });
	threads_coordinate.push_back({ 3.9, 7.2, 0.0 });
	threads_coordinate.push_back({ 3.7, 8.9, 0.0 });
	threads_coordinate.push_back({ 4.2, 5.4, 0.0 });

	//23~
	threads_coordinate.push_back({ 3.6, 4.9, 0.0 });
	threads_coordinate.push_back({ 4.4, 5.0, 0.0 });
	threads_coordinate.push_back({ 4.9, 5.7, 0.0 });
	threads_coordinate.push_back({ 5.0, 6.1, 0.0 });
	threads_coordinate.push_back({ 4.5, 2.8, 0.0 });

	//28~
	threads_coordinate.push_back({ 4.6, 4.3, 0.0 });
	threads_coordinate.push_back({ 4.8, 8.3, 0.0 });
	threads_coordinate.push_back({ 4.7, 6.8, 0.0 });
	threads_coordinate.push_back({ 5.1, 8.7, 0.0 });
	threads_coordinate.push_back({ 5.8, 4.0, 0.0 });

	//33~
	threads_coordinate.push_back({ 5.4, 8.8, 0.0 });
	threads_coordinate.push_back({ 5.7, 6.9, 0.0 });
	threads_coordinate.push_back({ 5.3, 8.1, 0.0 });
	threads_coordinate.push_back({ 5.6, 4.5, 0.0 });
	threads_coordinate.push_back({ 5.9, 6.5, 0.0 });

	//38~
	threads_coordinate.push_back({ 5.2, 5.3, 0.0 });
	threads_coordinate.push_back({ 5.5, 3.3, 0.0 });
	threads_coordinate.push_back({ 6.7, 8.6, 0.0 });
	threads_coordinate.push_back({ 6.0, 7.9, 0.0 });
	threads_coordinate.push_back({ 6.1, 4.6, 0.0 });

	//43~
	threads_coordinate.push_back({ 6.5, 5.6, 0.0 });
	threads_coordinate.push_back({ 6.4, 6.4, 0.0 });
	threads_coordinate.push_back({ 6.2, 7.4, 0.0 });
	threads_coordinate.push_back({ 6.6, 3.9, 0.0 });
	threads_coordinate.push_back({ 6.3, 3.1, 0.0 });

	//48~
	threads_coordinate.push_back({ 7.1, 8.5, 0.0 });
	threads_coordinate.push_back({ 7.2, 4.8, 0.0 });
	threads_coordinate.push_back({ 6.8, 3.7, 0.0 });
	threads_coordinate.push_back({ 6.9, 8.0, 0.0 });
	threads_coordinate.push_back({ 7.4, 5.9, 0.0 });

	//53~
	threads_coordinate.push_back({ 7.0, 3.4, 0.0 });
	threads_coordinate.push_back({ 7.5, 6.2, 0.0 });
	threads_coordinate.push_back({ 7.3, 7.0, 0.0 });
	threads_coordinate.push_back({ 7.8, 4.7, 0.0 });
	threads_coordinate.push_back({ 7.7, 7.5, 0.0 });

	//58~
	threads_coordinate.push_back({ 8.3, 4.2, 0.0 });
	threads_coordinate.push_back({ 8.1, 5.2, 0.0 });
	threads_coordinate.push_back({ 7.9, 8.2, 0.0 });
	threads_coordinate.push_back({ 8.0, 9.1, 0.0 });
	threads_coordinate.push_back({ 8.2, 3.0, 0.0 });

	//63~
	threads_coordinate.push_back({ 7.6, 6.6, 0.0 });
	threads_coordinate.push_back({ 8.4, 6.3, 0.0 });
	threads_coordinate.push_back({ 8.9, 2.9, 0.0 });
	threads_coordinate.push_back({ 8.7, 7.3, 0.0 });
	threads_coordinate.push_back({ 8.8, 5.1, 0.0 });

	//68~
	threads_coordinate.push_back({ 8.6, 7.6, 0.0 });
	threads_coordinate.push_back({ 8.5, 9.0, 0.0 });
	threads_coordinate.push_back({ 9.1, 5.5, 0.0 });
	threads_coordinate.push_back({ 9.0, 3.6, 0.0 });

	//ソート
	//指向性画像補正時に，補正前の左（vd座標の∞方向）から補正するため
	std::sort(threads_coordinate.begin(), threads_coordinate.end(),
		[](const cv::Point3d& a, const cv::Point3d& b) {
			return a.y > b.y; // y座標の降順
		}
	);


	return threads_coordinate;
}

//糸座標の端点を2つセットで保存する関数
vector<cv::Point3d> setThreadEndsCoordinate(vector<cv::Point3d> thr_pos) {
	vector<cv::Point3d> ret;

	for (int i = 0; i < thr_pos.size(); i++) {
		//元の糸座標を追加
		ret.push_back(thr_pos[i]);

		//糸座標のz座標に天板間の距離分追加し，保存
		thr_pos[i].z += VD_HEIGHTS;
		ret.push_back(thr_pos[i]);
	}

	//for (int i = 0; i < ret.size(); i++) {
	//	cout << ret[i] << endl;
	//}

	return ret;
}

//糸の座標を加工する関数
//糸座標をproj座標系に変換
vector<cv::Point3d> changeThreadsCoordinate(vector<cv::Point3d> thr_pos) {

	vector<cv::Point3d> ret;
	cv::Mat thr_pos_tmp0(3, 1, CV_64FC1);
	cv::Mat thr_pos_tmp1(3, 1, CV_64FC1);
	cv::Mat thr_transform(3, 3, CV_64FC1);	//軸を入れ替える用の行列

	thr_transform = (cv::Mat_<double>(3, 3) << 0, -1, 0,
		0, 0, 1,
		1, 0, 0);

	for (int i = 0; i < thr_pos.size(); i++) {

		//arMarkerを貼り付けた位置の分ずらす
		thr_pos_tmp0.at<double>(0, 0) = thr_pos[i].x - (double)VD_ARMARKER_POS_X;
		thr_pos_tmp0.at<double>(1, 0) = thr_pos[i].y - (double)VD_ARMARKER_POS_Y;
		thr_pos_tmp0.at<double>(2, 0) = thr_pos[i].z - (double)VD_ARMARKER_POS_Z;

		//軸を入れ替える行列をかける
		thr_pos_tmp1 = thr_transform * thr_pos_tmp0;

		ret.push_back({ thr_pos_tmp1.at<double>(0, 0) , thr_pos_tmp1.at<double>(1, 0) , thr_pos_tmp1.at<double>(2, 0) });
	}

	//加工後の糸座標を表示
	cout << "加工後の糸の座標を表示" << endl;
	for (int i = 0; i < ret.size(); i++) {
		cout << ret[i] << endl;
	}

	return ret;
}



//-----カメラキャリブレーション-------------------------------------------------------------
//カメラキャリブレーションでの内部・外部パラメータを保存する構造体
struct parameters_cc {
	//内部パラメータcc用
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;

	//外部パラメータcc用
	vector<cv::Mat> rvecs;
	vector<cv::Mat> tvecs;

	//外部パラメータcc用
	//cv::Mat rvec_RTvec;
	//cv::Mat tvec_RTvec;
};

parameters_cc getCcParameters() {
	parameters_cc p;
	cv::Mat camMat(3, 3, CV_64FC1);
	cv::Mat dc(1, 5, CV_64FC1);

	string filename = "inParams.json";
	ifstream ifs(filename.c_str());

	if (ifs.good()) {	//good()はファイルが無い時に0になる
		json j;
		ifs >> j;

		camMat.at<double>(0, 0) = j["cameraMatrix"][0][0];
		camMat.at<double>(0, 1) = j["cameraMatrix"][0][1];
		camMat.at<double>(0, 2) = j["cameraMatrix"][0][2];

		camMat.at<double>(1, 0) = j["cameraMatrix"][1][0];
		camMat.at<double>(1, 1) = j["cameraMatrix"][1][1];
		camMat.at<double>(1, 2) = j["cameraMatrix"][1][2];

		camMat.at<double>(2, 0) = j["cameraMatrix"][2][0];
		camMat.at<double>(2, 1) = j["cameraMatrix"][2][1];
		camMat.at<double>(2, 2) = j["cameraMatrix"][2][2];

		dc.at<double>(0, 0) = j["distCoeffs"][0];
		dc.at<double>(0, 1) = j["distCoeffs"][1];
		dc.at<double>(0, 2) = j["distCoeffs"][2];
		dc.at<double>(0, 3) = j["distCoeffs"][3];
		dc.at<double>(0, 4) = j["distCoeffs"][4];
	}
	else {
		cout << "内部パラメータを保存したjsonファイルが見つかりません" << endl;
		exit(-1);
	}

	p.cameraMatrix = camMat;
	p.distCoeffs = dc;

	return p;
}



//-----ArUcoマーカー検出-----------------------------------------------------------
//estimatePoseSingleMarkersの外部パラメータを保存する構造体
struct parameters_epsm {
	vector<cv::Mat> rvecs;
	vector<cv::Mat> tvecs;
};

//estimatePoseSingleMarkersの代わりの関数
//arMarkerの角の画像座標・大きさから，arMarkerを原点とした外部パラメータを計算
//参考：https://stackoverflow.com/questions/75750177/solve-pnp-or-estimate-pose-single-markers-which-is-better
parameters_epsm my_estimatePoseSingleMarkers(vector<vector<cv::Point2f>> corners, double arMarker_size, parameters_cc p) {
	parameters_epsm p_epsm;

	//arMarkerの四つ角の世界座標を設定，arMarkerの中心が原点
	vector<cv::Point3d> marker_points;

	//arMarkerの中心を世界座標系原点とする
	marker_points.push_back({ -arMarker_size / (double)2.0,		-arMarker_size / (double)2.0,	(double)0.0 });	//左上
	marker_points.push_back({ arMarker_size / (double)2.0,		-arMarker_size / (double)2.0,	(double)0.0 });	//右上
	marker_points.push_back({ arMarker_size / (double)2.0,		arMarker_size / (double)2.0,		(double)0.0 });	//右下
	marker_points.push_back({ -arMarker_size / (double)2.0,		arMarker_size / (double)2.0,		(double)0.0 });	//左下

	cv::Mat rtvec_transform(3, 3, CV_64FC1);	//軸を入れ替える用の行列
	rtvec_transform = (cv::Mat_<double>(3, 3) << 0, 1, 0,
		0, 0, 1,
		1, 0, 0);

	for (int i = 0; i < corners.size(); i++) {
		//cout << "corners[i]の要素数" << endl;
		//cout << corners[i].size() << endl;

		//デフォルトの計算方法
		//cv::Mat rvecs_tmp;
		//cv::Mat tvecs_tmp;
		//cv::solvePnP(marker_points, corners[i], p.cameraMatrix, p.distCoeffs, rvecs_tmp, tvecs_tmp);
		//p_epsm.rvecs.push_back(rvecs_tmp);
		//p_epsm.tvecs.push_back(tvecs_tmp);

		//平面物体の計測に特化した計算方法
		vector<cv::Mat> rvecs_tmp;
		vector<cv::Mat> tvecs_tmp;
		//cv::solvePnP(marker_points, corners[i], p.cameraMatrix, p.distCoeffs, rvecs_tmp, tvecs_tmp, cv::SOLVEPNP_IPPE);]
		cv::solvePnPGeneric(marker_points, corners[i], p.cameraMatrix, p.distCoeffs, rvecs_tmp, tvecs_tmp, false, cv::SOLVEPNP_IPPE);

		for (int j = 0; j < rvecs_tmp.size(); j++) {
			//回転ベクトルを回転行列に変換
			cv::Mat rmat_tmp;
			cv::Rodrigues(rvecs_tmp[j], rmat_tmp);

			//回転行列からz軸の向きを表す成分の抜き出す
			cv::Vec3d z_axis_tmp(rmat_tmp.at<double>(0, 2), rmat_tmp.at<double>(1, 2), rmat_tmp.at<double>(2, 2));

			//z軸の向きが負になる（マーカーの紙を見下ろすように撮影している）場合
			if (z_axis_tmp[2] > 0) {
				cv::Mat rvec = rtvec_transform * rvecs_tmp[j];
				cv::Mat tvec = rtvec_transform * tvecs_tmp[j];

				//cout << "回転ベクトル" << endl;
				//cout << rvecs_tmp[j] << endl;
				//cout << rvec << endl;
				//cout << "並進ベクトル" << endl;
				//cout << tvecs_tmp[j] << endl;
				//cout << tvec << endl;

				p_epsm.rvecs.push_back(rvec);
				p_epsm.tvecs.push_back(tvec);
				break;
			}
		}
	}

	return p_epsm;
}

//arMarkerの位置に3次元座標を表示する関数
void my_drawAxis(cv::Mat img, parameters_epsm p_epsm, parameters_cc p, int ids0, int ids1) {

	double arMarker_size = ARMARKER_SIZE;

	vector<cv::Point3d> axis;
	axis.push_back(cv::Point3d(0, 0, 0));
	axis.push_back(cv::Point3d(arMarker_size * 0.5f, 0, 0));
	axis.push_back(cv::Point3d(0, arMarker_size * 0.5f, 0));
	axis.push_back(cv::Point3d(0, 0, arMarker_size * 0.5f));

	vector<cv::Point2f> imgPoints;
	cv::projectPoints(axis, p_epsm.rvecs[ids0], p_epsm.tvecs[ids0], p.cameraMatrix, p.distCoeffs, imgPoints);
	cv::line(img, imgPoints[0], imgPoints[1], cv::Scalar(0, 0, 255), 2); // X軸：赤
	cv::line(img, imgPoints[0], imgPoints[2], cv::Scalar(0, 255, 0), 2); // Y軸：緑
	cv::line(img, imgPoints[0], imgPoints[3], cv::Scalar(255, 0, 0), 2); // Z軸：青

	imgPoints.clear();
	cv::projectPoints(axis, p_epsm.rvecs[ids1], p_epsm.tvecs[ids1], p.cameraMatrix, p.distCoeffs, imgPoints);
	cv::line(img, imgPoints[0], imgPoints[1], cv::Scalar(0, 0, 255), 2); // X軸：赤
	cv::line(img, imgPoints[0], imgPoints[2], cv::Scalar(0, 255, 0), 2); // Y軸：緑
	cv::line(img, imgPoints[0], imgPoints[3], cv::Scalar(255, 0, 0), 2); // Z軸：青
}

//arMarkerのcornerの格納順を画像に表示する関数
void drawMarkerCornersWithOrder(
	cv::Mat& img,
	const std::vector<std::vector<cv::Point2f>>& corners,
	const std::vector<int>& ids)
{
	if (img.empty() || ids.empty() || corners.size() != ids.size()) return;

	const double s = std::max(1.0, std::min(img.cols, img.rows) / 800.0);
	const int line_th = int(2 * s);
	const int radius = int(4 * s);
	const double font_scale = 0.6 * s;
	const int font_th = std::max(1, int(2 * s));

	for (size_t idx = 0; idx < ids.size(); ++idx) {
		const int markerId = ids[idx];              // ← これが表示すべき “IDの値”
		const auto& c = corners[idx];               // ← corners は検出順 idx で参照
		if (c.size() < 4) continue;

		// 0→1→2→3→0 と結ぶ
		for (int k = 0; k < 4; ++k) {
			cv::line(img, c[k], c[(k + 1) % 4], cv::Scalar(0, 255, 0), line_th, cv::LINE_AA);
		}

		// 各頂点に点とインデックス
		for (int k = 0; k < 4; ++k) {
			cv::circle(img, c[k], radius, cv::Scalar(255, 0, 0), -1, cv::LINE_AA);
			cv::putText(img, std::to_string(k),
				c[k] + cv::Point2f(4.f * (double)s, -4.f * (double)s),
				cv::FONT_HERSHEY_SIMPLEX, font_scale,
				cv::Scalar(255, 0, 0), font_th, cv::LINE_AA);
		}

		// “ID:<値>” を 0番コーナー付近に表示
		const std::string label = "ID:" + std::to_string(markerId);  // ← ここ！
		const cv::Point2f pos = c[0] + cv::Point2f(6.f * (double)s, 18.f * (double)s);
		cv::putText(img, label, pos, cv::FONT_HERSHEY_SIMPLEX, font_scale,
			cv::Scalar(0, 0, 0), font_th + 2, cv::LINE_AA);
		cv::putText(img, label, pos, cv::FONT_HERSHEY_SIMPLEX, font_scale,
			cv::Scalar(255, 255, 255), font_th, cv::LINE_AA);

		// 回り方向の矢印を出したい場合
		// cv::arrowedLine(img, c[0], c[1], cv::Scalar(0,200,255), line_th, cv::LINE_AA, 0, 0.25);
	}
}

//ids1座標系をids0座標系に変換する回転行列を計算
cv::Mat calc_rmat_twoMarkers(parameters_epsm p, int ids0, int ids1) {

	//回転ベクトルを回転行列に変換
	cv::Mat R0_matrix;
	cv::Rodrigues(p.rvecs[ids0], R0_matrix);
	cv::Mat R1_matrix;
	cv::Rodrigues(p.rvecs[ids1], R1_matrix);

	//ids0に対するids1の相対的な回転行列を計算
	cv::Mat relativeRot = R0_matrix.t() * R1_matrix;

	return relativeRot;
}

//ids1座標系をids0座標系に変換する並進ベクトルを計算
cv::Mat calc_tvec_twoMarkers(parameters_epsm p, int ids0, int ids1) {

	//ids0の回転ベクトルを回転行列に変換
	cv::Mat R0_matrix;
	cv::Rodrigues(p.rvecs[ids0], R0_matrix);

	//ids0からids1の距離ベクトルを計算
	cv::Mat relativePos = R0_matrix.t() * (p.tvecs[ids1] - p.tvecs[ids0]);

	return relativePos;
}

//回転行列をdeg単位の回転ベクトルに変形する関数
cv::Mat myRodrigues(cv::Mat Rmat) {

	cv::Mat Rvec;
	cv::Rodrigues(Rmat, Rvec);
	Rvec *= (180.0 / CV_PI);

	return Rvec;
}

//make_projectionImg_projectiveの返り値をまとめる構造体
struct makeProjImg {
	cv::Mat projImg;
	vector<cv::Point2i> thr_pos_projImg;
	int lineWidth;
	cv::Mat R_adj;
	cv::Mat t_adj;
	int lineLength;
};

//プロジェクターの水平画角 AOVx [rad] と画像サイズから，仮の内部パラメータを生成
// fx = fy を仮定、主点は画像中心、歪みゼロ
parameters_cc build_proj_inParams_FromAOV(
	double AOVx_rad,		//水平方向の画角
	const cv::Size& size	//プロジェクターの解像度
) {
	const double cx = (size.width - 1) * 0.5;
	const double cy = (size.height - 1) * 0.5;
	const double fx = cx / std::tan(0.5 * AOVx_rad);
	const double fy = fx; // 簡易仮定

	parameters_cc ret;

	//カメラのFOVから内部パラを設定
	ret.cameraMatrix = (cv::Mat_<double>(3, 3) <<
		fx, 0, cx,
		0, fy, cy,
		0, 0, 1);

	//レンズ歪みは0で初期化
	ret.distCoeffs = cv::Mat::zeros(1, 5, CV_64F);

	//cout << endl;
	//cout << "projの内部パラメーター" << endl;
	//cout << ret.cameraMatrix << endl;
	//cout << ret.distCoeffs << endl;
	//cout << endl;

	return ret;
}

//プロジェクタをピンホールカメラとして扱い，糸座標を厳密に射影することで，投影画像を生成
makeProjImg make_projectionImg_projective(
	const std::vector<cv::Point3d>& thr_pos,
	const cv::Mat& R_relative, const cv::Mat& t_relative,				//2つのマーカーの相対回転行列・相対並進ベクトル
	const parameters_cc projP,	//プロジェクターの内部パラメーター
	const cv::Size& projImgSize,
	const cv::Scalar& backColor,
	const vector<cv::Scalar> lineColor,
	int lineWidth
) {
	// 出力バッファを背景色で初期化
	cv::Mat projImg(projImgSize, CV_8UC3, backColor);

	makeProjImg mpi;

	//入力チェック
	if (R_relative.rows != 3 && R_relative.cols != 3 && R_relative.type() != 6) {
		cout << "make_projectionImg_projective : 2つのマーカーの相対的な回転行列が不正です" << endl;
		exit(-1);
	}
	if (t_relative.rows != 3 && t_relative.type() != 6) {
		cout << "make_projectionImg_projective : 2つのマーカーの相対的な並進ベクトルが不正です" << endl;
		exit(-1);
	}
	if (projP.cameraMatrix.rows != 3 && projP.cameraMatrix.cols != 3) {
		cout << "make_projectionImg_projective : プロジェクターの内部パラメーターが不正です" << endl;
		exit(-1);
	}
	if (projImgSize.width <= 0 && projImgSize.height <= 0) {
		cout << "make_projectionImg_projective : 投影画像のサイズが不正です" << endl;
		exit(-1);
	}

	//回転行列を回転ベクトルへ変換（projectPointsは rvec / tvec を要求）
	cv::Mat rvec;
	cv::Rodrigues(R_relative, rvec); // 3x1, CV_64F が得られる

	// Z<=0（= プロジェクタ背面/投影不能）を除外したい場合は、事前に外部パラメータで前向き判定
	// ここでは簡単のため projectPoints の前に自前で前向き判定を行う
	std::vector<cv::Point3d> pts_front;
	pts_front.reserve(thr_pos.size());
	{
		for (const auto& X : thr_pos) {

			//proj座標系で糸の座標を計算
			cv::Mat v(3, 1, CV_64F);
			v.at<double>(0, 0) = X.x;
			v.at<double>(1, 0) = X.y;
			v.at<double>(2, 0) = X.z;

			cv::Mat Xp = R_relative * v + t_relative;
			//cout << "proj座標系の糸座標	" << endl;
			//cout << Xp << endl;

			//projの前方に糸があるかを判定
			if (Xp.at<double>(2, 0) > 1e-6f) {	//閾値は0より少し大きい値
				pts_front.emplace_back(X);
			}
			else {
				cout << "投影範囲に入っていない糸があります" << endl;
				exit(-1);
			}
		}
	}

	//糸が一本も前方にない時，空の画像を返す
	if (pts_front.empty()) {
		mpi.projImg = projImg;
	}

	//vd座標系の糸座標をproj座標系に変換し，投影画像平面に射影
	std::vector<cv::Point2d> uv;	//糸座標の射影後の座標
	cv::projectPoints(pts_front, rvec, t_relative, projP.cameraMatrix, projP.distCoeffs, uv);
	//for (int i = 0; i < uv.size(); i++) {
	//	cout << uv[i] << endl;
	//}

	// 射影結果の u(=x) 列に縦線を描画（画像範囲に収まるもののみ）
	// 同じ列が近接して重複する場合のために丸め＋ユニーク化
	std::vector<cv::Point2i> thr_pos_projImg;	//投影画像上での糸座標（整数値）
	thr_pos_projImg.reserve(uv.size());

	for (const auto& p : uv) {

		if (cv::checkRange(cv::Mat(p)) && std::isfinite(p.x) && std::isfinite(p.y)) {	//射影後の糸座標がNaN，∞でないかチェック
			//射影後の糸座標を整数値にするため四捨五入
			int x = (int)std::lround(p.x);
			int y = (int)std::lround(p.y);

			thr_pos_projImg.push_back({ x, y });
		}
	}
	if (thr_pos_projImg.empty()) {	//投影範囲内に糸が無い場合
		mpi.projImg = projImg;
	}
	if (thr_pos_projImg.size() % 2 != 0) {
		cout << "糸の端点座標が上下ペアで保存されていません" << endl;
		exit(-1);
	}

	//cout << "糸の端点座標" << endl;
	//for (int i = 0; i < thr_pos_projImg.size(); i++) {
	//	cout << thr_pos_projImg[i] << endl;
	//}

	//投影画像中に直線を描画
	cv::Rect imgRect(0, 0, projImg.cols, projImg.rows);  // 画像範囲
	for (int i = 0; i < thr_pos_projImg.size(); i += 2) {
		if (cv::clipLine(imgRect, thr_pos_projImg[i], thr_pos_projImg[i + 1])) {	//糸の上端下端を結ぶ直線が投影範囲（矩形）と交差するときtrueを返す
			cv::line(projImg, thr_pos_projImg[i], thr_pos_projImg[i + 1], lineColor[(i / 2) % lineColor.size()], lineWidth, cv::LINE_AA);
		}
	}

	mpi.projImg = projImg;
	mpi.thr_pos_projImg = thr_pos_projImg;
	mpi.lineWidth = lineWidth;

	return mpi;
}

//プロジェクタをピンホールカメラとして扱い，糸座標を厳密に射影することで，投影画像を生成
makeProjImg make_projectionImg_projective_individually(
	const std::vector<cv::Point3d>& thr_pos,
	const cv::Mat& R_relative, const cv::Mat& t_relative,				//2つのマーカーの相対回転行列・相対並進ベクトル
	const parameters_cc projP,	//プロジェクターの内部パラメーター
	const cv::Size& projImgSize,
	const cv::Scalar& backColor,
	const vector<cv::Scalar> lineColor,
	int lineWidth,
	int selectedThread,
	const cv::Mat& R_individually, const cv::Mat& t_individually,		//調節対象の回転行列・相対並進ベクトル
	vector<cv::Point2i> uv												//糸座標の射影後の座標
) {
	// 出力バッファを背景色で初期化
	cv::Mat projImg(projImgSize, CV_8UC3, backColor);

	makeProjImg mpi;

	//入力チェック
	if (R_relative.rows != 3 && R_relative.cols != 3 && R_relative.type() != 6) {
		cout << "make_projectionImg_projective : 2つのマーカーの相対的な回転行列が不正です" << endl;
		exit(-1);
	}
	if (t_relative.rows != 3 && t_relative.type() != 6) {
		cout << "make_projectionImg_projective : 2つのマーカーの相対的な並進ベクトルが不正です" << endl;
		exit(-1);
	}
	if (projP.cameraMatrix.rows != 3 && projP.cameraMatrix.cols != 3) {
		cout << "make_projectionImg_projective : プロジェクターの内部パラメーターが不正です" << endl;
		exit(-1);
	}
	if (projImgSize.width <= 0 && projImgSize.height <= 0) {
		cout << "make_projectionImg_projective : 投影画像のサイズが不正です" << endl;
		exit(-1);
	}

	//回転行列を回転ベクトルへ変換（projectPointsは rvec / tvec を要求）
	cv::Mat rvec;
	cv::Rodrigues(R_relative, rvec);	// 3x1, CV_64F が得られる
	cv::Mat rvec_individually;			//調整対象用
	cv::Rodrigues(R_individually, rvec_individually);

	//前向き判定、調整対象用
	std::vector<cv::Point3d> pts_individually;
	{
		for (int i = 0; i < 2; i++) {
			//proj座標系で糸の座標を計算
			cv::Mat v(3, 1, CV_64F);
			v.at<double>(0, 0) = thr_pos[selectedThread + i].x;
			v.at<double>(1, 0) = thr_pos[selectedThread + i].y;
			v.at<double>(2, 0) = thr_pos[selectedThread + i].z;

			cv::Mat Xp = R_individually * v + t_individually;
			//cout << "proj座標系の糸座標	" << endl;
			//cout << Xp << endl;

			//projの前方に糸があるかを判定
			if (Xp.at<double>(2, 0) > 1e-6f) {	//閾値は0より少し大きい値
				pts_individually.emplace_back(thr_pos[selectedThread + i]);
			}
			else {
				cout << "投影対象の糸が投影範囲に入っていません" << endl;
				exit(-1);
			}
		}
	}

	//投影画像平面に射影、投影対象用
	std::vector<cv::Point2d> uv_individually;	//射影後の座標、調整対象用
	cv::projectPoints(pts_individually, rvec_individually, t_individually, projP.cameraMatrix, projP.distCoeffs, uv_individually);

	//uvのうち、調整対象の糸座標を入れ替え
	uv[selectedThread] = uv_individually[0];
	uv[selectedThread + 1] = uv_individually[1];


	// 射影結果の u(=x) 列に縦線を描画（画像範囲に収まるもののみ）
	// 同じ列が近接して重複する場合のために丸め＋ユニーク化
	std::vector<cv::Point2i> thr_pos_projImg;	//投影画像上での糸座標（整数値）
	thr_pos_projImg.reserve(uv.size());

	for (const auto& p : uv) {

		if (cv::checkRange(cv::Mat(p)) && std::isfinite(p.x) && std::isfinite(p.y)) {	//射影後の糸座標がNaN，∞でないかチェック
			//射影後の糸座標を整数値にするため四捨五入
			int x = (int)std::lround(p.x);
			int y = (int)std::lround(p.y);

			thr_pos_projImg.push_back({ x, y });
		}
	}
	if (thr_pos_projImg.empty()) {	//投影範囲内に糸が無い場合
		mpi.projImg = projImg;
	}
	if (thr_pos_projImg.size() % 2 != 0) {
		cout << "糸の端点座標が上下ペアで保存されていません" << endl;
		exit(-1);
	}

	//cout << "糸の端点座標" << endl;
	//for (int i = 0; i < thr_pos_projImg.size(); i++) {
	//	cout << thr_pos_projImg[i] << endl;
	//}

	//投影画像中に直線を描画
	cv::Rect imgRect(0, 0, projImg.cols, projImg.rows);  // 画像範囲
	for (int i = 0; i < thr_pos_projImg.size(); i += 2) {
		if (cv::clipLine(imgRect, thr_pos_projImg[i], thr_pos_projImg[i + 1])) {	//糸の上端下端を結ぶ直線が投影範囲（矩形）と交差するときtrueを返す

			if (i == selectedThread) {
				cv::line(projImg, thr_pos_projImg[i], thr_pos_projImg[i + 1], lineColor[0], lineWidth, cv::LINE_AA);
			}
			else {
				cv::line(projImg, thr_pos_projImg[i], thr_pos_projImg[i + 1], lineColor[1], lineWidth, cv::LINE_AA);
			}
		}
	}

	mpi.projImg = projImg;
	mpi.thr_pos_projImg = thr_pos_projImg;
	mpi.lineWidth = lineWidth;

	return mpi;
}

//各フレームで計算した相対的な回転行列・並進ベクトルを保存する構造体
struct result_test_arMarkerDetect {
	cv::Mat R_relative;
	cv::Mat t_relative;
};

//静止画像から2つのarMarkerを検出
result_test_arMarkerDetect test_arMarkerDetect(cv::aruco::Dictionary dictionary, parameters_cc p, string filename) {

	//arucoDetectorオブジェクトを作成
	cout << "変数・オブジェクト作成開始" << endl;
	//デフォルト
	cv::aruco::DetectorParameters Detector_p = cv::aruco::DetectorParameters();
	cv::aruco::ArucoDetector detector = cv::aruco::ArucoDetector(dictionary, Detector_p);

	cv::Mat cap_img;
	vector<vector<cv::Point2f>> corners;			//arMarkerの角の画像座標を左上から反時計回りで格納
	vector<int> ids;								//arMarkerのIDを格納
	vector<vector<cv::Point2f>> rejectedImgPoints;	//デバッグのために適切なパターンを持たない正方形の画像座標を格納

	//arMarkerの大きさ，枚数を保存
	double arMarker_size = ARMARKER_SIZE;
	int arMarker_num = ARMARKER_NUM;

	//カメラ画像の読み込みを繰り返し
	cout << "画像の読み込み開始" << endl;

	//画像入力
	if (true == empty(cv::imread(filename))) {
		cout << "test画像が見つかりません" << endl;
		exit(-1);
	}
	else {
		cap_img = cv::imread(filename);
	}

	//オブジェクトを作成
	detector.cv::aruco::ArucoDetector::detectMarkers(cap_img, corners, ids, rejectedImgPoints);
	//for (int i = 0; i < corners.size(); i++) {
	//	cout << corners[i] << endl;
	//}

	//arMarkerの情報を画像に描画
	//cv::aruco::drawDetectedMarkers(cap_img, corners, ids);
	drawMarkerCornersWithOrder(cap_img, corners, ids);

	double scale = 0.35;
	cv::Mat resized_cap_img;
	cv::resize(cap_img, resized_cap_img, cv::Size((int)cap_img.cols * scale, (int)cap_img.rows * scale));
	cv::imshow("detectResult", resized_cap_img);
	//cv::waitKey(0);

	//プロジェクターに貼るArUcoマーカーのずれを表す行列
	cv::Mat offset_proj = (cv::Mat_<double>(3, 1) <<
		PROJ_ARMARKER_POS_X,
		PROJ_ARMARKER_POS_Y,
		PROJ_ARMARKER_POS_Z);

	//認識するarMarkerの数を選択
	result_test_arMarkerDetect ret;
	if (corners.size() == 2) {
		if ((ids[0] == PROJ_ARMARKER_ID && ids[1] == VD_ARMARKER_ID) || (ids[0] == VD_ARMARKER_ID && ids[1] == PROJ_ARMARKER_ID)) {

			//arMarkerを映したカメラの外部パラメータを計算
			parameters_epsm p_epsm;
			p_epsm = my_estimatePoseSingleMarkers(corners, arMarker_size, p);

			//idsの値が小さい順に順序を記録
			vector<int> ids_order;
			for (int i = 0; i < arMarker_num; i++) {	//arMarkerの枚数分繰り返す
				for (int j = 0; j < ids.size(); j++) {	//idsのsize分繰り返す
					if (ids[j] == i) {
						ids_order.push_back(j);
						break;
					}
				}
			}

			//arMarkerに3次元座標軸を表示
			//my_drawAxis(cap_img, p_epsm, p, ids_order[0], ids_order[1]);
			//arMarkerのx軸：右が正，y軸：上が正，z軸：手前が正
			cv::Mat R_relative = calc_rmat_twoMarkers(p_epsm, ids_order[0], ids_order[1]);
			cv::Mat T_relative = calc_tvec_twoMarkers(p_epsm, ids_order[0], ids_order[1]);

			//プロジェクターに貼るArUcoマーカーのずれを反映
			T_relative += R_relative * offset_proj;

			cout << endl;
			cout << "相対的な回転ベクトル[deg]" << endl;
			cv::Mat RvecTmp = myRodrigues(R_relative);
			cout << RvecTmp << endl;
			cout << "相対的な並進ベクトル[cm]" << endl;
			cout << T_relative << endl;
			cout << endl;

			ret.R_relative = R_relative;
			ret.t_relative = T_relative;
		}
		else {
			cout << "IDが0と1以外のArUcoマーカーが検出されました" << endl;
			exit(-1);
		}
	}
	else {
		cout << "test画像からArUcoマーカーを検出できませんでした" << endl;
		exit(-1);
	}

	//cv::imwrite("cap_img.bmp", cap_img);

	return ret;
}



//-----姿勢の手動微調整UI-----------------------------------------------------
//回転角から回転行列を作成する関数
static cv::Mat so3Exp_deg(double rx_deg, double ry_deg, double rz_deg) {
	// roll=X, pitch=Y, yaw=Z（度→ラジアン）; 左乗で R' = Rz * Ry * Rx * R_base
	const double rx = rx_deg * CV_PI / 180.0;
	const double ry = ry_deg * CV_PI / 180.0;
	const double rz = rz_deg * CV_PI / 180.0;
	const double cx = cos(rx), sx = sin(rx);
	const double cy = cos(ry), sy = sin(ry);
	const double cz = cos(rz), sz = sin(rz);
	cv::Mat Rx = (cv::Mat_<double>(3, 3) <<
		1, 0, 0,
		0, cx, -sx,
		0, sx, cx);
	cv::Mat Ry = (cv::Mat_<double>(3, 3) <<
		cy, 0, sy,
		0, 1, 0,
		-sy, 0, cy);
	cv::Mat Rz = (cv::Mat_<double>(3, 3) <<
		cz, -sz, 0,
		sz, cz, 0,
		0, 0, 1);
	return Rz * Ry * Rx; // Z->Y->X の順で左乗
}

//スライダーで投影画像を微調整する関数
makeProjImg interactiveAdjustProjection(
	const std::vector<cv::Point3d>& thr_pos,
	const cv::Mat& R_base, const cv::Mat& t_base,
	parameters_cc projP,
	vector<cv::Scalar> lineColor
) {
	//スライダーは整数なのでスケールして扱う（0.01単位）
	auto s2dbl = [](int v) { return v / SLIDER_SCALE; };

	//トラックバー関連の変数
	int tx_s = 0, ty_s = 0, tz_s = 0;
	int roll_s = 0, pitch_s = 0, yaw_s = 0;
	const int SminShift = -200, SmaxShift = 200;	//並進移動トラックバーの上限下限
	const int SminRot = -400, SmaxRot = 400;		//回転移動トラックバーの上限下限

	//直線の太さ
	int width_s = 1;
	const int SminWidth = 1, SmaxWidth = 256;

	//直線の長さ
	int length_s = 100;
	const int SminLength = 1, SmaxLength = 100;

	//背景色
	int back_s = 0;
	const int SminBack = 0, SmaxBack = 255;

	//スライダーのウィンドウを作成
	cv::namedWindow("Tune Projection", cv::WINDOW_NORMAL);
	cv::resizeWindow("Tune Projection", 1280, 800);
	cv::createTrackbar("tx ", "Tune Projection", &tx_s, SmaxShift); cv::setTrackbarMin("tx ", "Tune Projection", SminShift);
	cv::createTrackbar("ty ", "Tune Projection", &ty_s, SmaxShift); cv::setTrackbarMin("ty ", "Tune Projection", SminShift);
	cv::createTrackbar("tz ", "Tune Projection", &tz_s, SmaxShift); cv::setTrackbarMin("tz ", "Tune Projection", SminShift);
	cv::createTrackbar("roll ", "Tune Projection", &roll_s, SmaxRot); cv::setTrackbarMin("roll ", "Tune Projection", SminRot);
	cv::createTrackbar("pitch ", "Tune Projection", &pitch_s, SmaxRot); cv::setTrackbarMin("pitch ", "Tune Projection", SminRot);
	cv::createTrackbar("yaw ", "Tune Projection", &yaw_s, SmaxRot); cv::setTrackbarMin("yaw ", "Tune Projection", SminRot);
	cv::createTrackbar("width ", "Tune Projection", &width_s, SmaxWidth); cv::setTrackbarMin("width ", "Tune Projection", SminWidth);
	cv::createTrackbar("length ", "Tune Projection", &length_s, SmaxLength); cv::setTrackbarMin("length ", "Tune Projection", SminLength);
	cv::createTrackbar("backColor ", "Tune Projection", &back_s, SmaxBack); cv::setTrackbarMin("backColor ", "Tune Projection", SminBack);

	cout << endl;
	cout << "コマンドリスト" << endl;
	cout << "Enter	プログラム終了" << endl;
	cout << "s	現在の糸座標を保存" << endl;
	cout << "r	補正をリセット" << endl;
	cout << endl;

	makeProjImg ret;

	while (true) {

		// 現在のΔを実数化
		const double tx = s2dbl(tx_s);
		const double ty = s2dbl(ty_s);
		const double tz = s2dbl(tz_s);
		const double rX = s2dbl(roll_s);
		const double rY = s2dbl(pitch_s);
		const double rZ = s2dbl(yaw_s);
		const int wd = width_s;
		const double ln = length_s / 100.0;
		const int bc = back_s;

		// ΔR を左乗、Δt を加算
		cv::Mat R_delta = so3Exp_deg(rX, rY, rZ);
		cv::Mat R_adj = R_delta * R_base;
		cv::Mat t_adj = t_base.clone();
		t_adj.at<double>(0) += tx;
		t_adj.at<double>(1) += ty;
		t_adj.at<double>(2) += tz;

		//糸の3次元座標を糸の中心方向にスケーリング
		std::vector<cv::Point3d> thr_pos_scaled;
		thr_pos_scaled.reserve(thr_pos.size());
		for (int i = 0; i < thr_pos.size(); i += 2) {
			cv::Point3d thr_top = thr_pos[i];
			cv::Point3d thr_bottom = thr_pos[i + 1];
			cv::Point3d thr_center = (thr_top + thr_bottom) * 0.5;	//糸の中心を表す3次元座標
			cv::Point3d d = thr_top - thr_bottom;					//糸の下端から上端を表すベクトル

			cv::Point3d thr_top_s = thr_center + 0.5 * ln * d;
			cv::Point3d thr_bottom_s = thr_center - 0.5 * ln * d;

			thr_pos_scaled.push_back(thr_top_s);
			thr_pos_scaled.push_back(thr_bottom_s);
		}


		// 調整後の投影画像
		//makeProjImg resMeasure = make_projectionImg(thr_pos, R_adj, t_adj, backColor, lineColor, lineWidth);	//yawのみ対応
		makeProjImg resMeasure = make_projectionImg_projective(thr_pos_scaled, R_adj, t_adj, projP, cv::Size(PROJ_IMG_WIDTH, PROJ_IMG_HEIGHT), cv::Scalar(bc, bc, bc), lineColor, wd);	//roll, yaw, pitchに対応

		// テキストオーバーレイ
		cv::Mat disp = resMeasure.projImg.clone();
		char buf[256];
		std::snprintf(buf, sizeof(buf),
			"(x, y, z)[cm]=[%.2f, %.2f, %.2f]", tx, ty, tz);
		cv::putText(disp, buf, { 20, 40 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 40,40,40 }, 3, cv::LINE_AA);
		cv::putText(disp, buf, { 20, 40 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 240,240,240 }, 1, cv::LINE_AA);

		char buf2[256];
		std::snprintf(buf2, sizeof(buf2),
			"(roll, pitch, yaw)[deg]=[%.2f, %.2f, %.2f]", rX, rY, rZ);
		cv::putText(disp, buf2, { 20, 90 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 40,40,40 }, 3, cv::LINE_AA);
		cv::putText(disp, buf2, { 20, 90 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 240,240,240 }, 1, cv::LINE_AA);

		char buf3[256];
		std::snprintf(buf3, sizeof(buf3),
			"line width[px]=%d  length[%%]=%d", wd, length_s);
		cv::putText(disp, buf3, { 20, 140 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 40,40,40 }, 3, cv::LINE_AA);
		cv::putText(disp, buf3, { 20, 140 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 240,240,240 }, 1, cv::LINE_AA);

		char buf4[256];
		std::snprintf(buf4, sizeof(buf4),
			"backColor=(%d, %d, %d)", bc, bc, bc);
		cv::putText(disp, buf4, { 20, 190 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 40,40,40 }, 3, cv::LINE_AA);
		cv::putText(disp, buf4, { 20, 190 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 240,240,240 }, 1, cv::LINE_AA);

		//画像を表示
		cv::imshow("Tune Projection", disp);

		cv::namedWindow("Tuned img", cv::WINDOW_NORMAL);
		cv::setWindowProperty("Tuned img", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
		cv::moveWindow("Tuned img", 1920, -100);
		cv::imshow("Tuned img", resMeasure.projImg);

		//コマンド
		int key = cv::waitKey(30);
		if (key == 13) {	// Enterで閉じる
			ret.lineWidth = 0;

			cv::Mat projImgTmp(cv::Size(PROJ_IMG_WIDTH, PROJ_IMG_HEIGHT), CV_8UC3, cv::Scalar(0, 0, 0));
			ret.projImg = projImgTmp;

			vector<cv::Point2i> thr_pos_projImg_tmp;
			ret.thr_pos_projImg = thr_pos_projImg_tmp;

			cout << "補正情報を保存せずに終了" << endl;
			cout << endl;

			break;
		}
		if (key == 's') {	// sで現在の状態を保存し出力
			//背景を黒にして，投影画像を作成
			ret = make_projectionImg_projective(thr_pos_scaled, R_adj, t_adj, projP, cv::Size(PROJ_IMG_WIDTH, PROJ_IMG_HEIGHT), cv::Scalar(0, 0, 0), lineColor, wd);

			//調整後の回転・並進ベクトル、光線の長さを保存
			ret.R_adj = R_adj;
			ret.t_adj = t_adj;
			ret.lineLength = length_s;

			cout << endl;

			cv::imshow("Tuned img", ret.projImg);
			cv::imwrite("projImg_threads_tuned.bmp", ret.projImg);
			//cout << "投影画像を保存: projection_tuned.png" << endl;

			cout << "直線の幅 : " << ret.lineWidth << "[px]" << endl;
			cout << "投影画像平面上の糸の座標" << endl;
			for (int i = 0; i < ret.thr_pos_projImg.size(); i++) {
				cout << ret.thr_pos_projImg[i] << endl;
			}
			cout << endl;

			//cv::waitKey(0);
			break;
		}
		if (key == 'r') {	// rで全リセット（スライダーと糸座標）
			//並進ベクトル・回転行列をリセット
			R_adj = R_base;
			t_adj = t_base;

			// トラックバー位置を中央（0）に戻す
			tx_s = ty_s = tz_s = roll_s = pitch_s = yaw_s = 0;
			width_s = 1;
			length_s = 100;
			back_s = 127;
			cv::setTrackbarPos("tx ", "Tune Projection", tx_s);
			cv::setTrackbarPos("ty ", "Tune Projection", ty_s);
			cv::setTrackbarPos("tz ", "Tune Projection", tz_s);
			cv::setTrackbarPos("roll ", "Tune Projection", roll_s);
			cv::setTrackbarPos("pitch ", "Tune Projection", pitch_s);
			cv::setTrackbarPos("yaw ", "Tune Projection", yaw_s);
			cv::setTrackbarPos("width ", "Tune Projection", width_s);
			cv::setTrackbarPos("length ", "Tune Projection", length_s);
			cv::setTrackbarPos("backColor ", "Tune Projection", back_s);

			std::cout << "補正をリセット" << endl;
		}
	}

	cv::destroyWindow("Tune Projection");
	return ret;
}

//vdの糸配置を画像で表示、調整対象の糸をハイライト
cv::Mat makeImg_threadArrengiment(const std::vector<cv::Point3d>& thr_pos, int selectedThread) {

	//糸配置を表示する画像
	cv::Mat threadArrengiment = cv::Mat::zeros(400, 400, CV_8UC3);

	double pixel_size = 0.02; //画像の1pixel = 0.01cm(0.1mm)
	for (int i = 0; i < thr_pos.size(); i = i + 2) {

		cv::Point center = { int((thr_pos[i].x + VD_ARMARKER_POS_Y - 2.0) / pixel_size), threadArrengiment.rows - int((thr_pos[i].z + VD_ARMARKER_POS_X - 2.0) / pixel_size) };

		if (i == selectedThread) {
			cv::circle(threadArrengiment, center, 3, cv::Scalar(0, 0, 255), -1, cv::LINE_8);
		}
		else {
			cv::circle(threadArrengiment, center, 1, cv::Scalar(255, 255, 255), -1, cv::LINE_8);
		}
	}

	return threadArrengiment;
}

//スライダーで投影画像を微調整する関数
//糸のインデックスを指定し、1本単位で調整
makeProjImg interactiveAdjustProjection_individually(
	const std::vector<cv::Point3d>& thr_pos,	//糸座標
	parameters_cc projP,						//プロジェクターの内部パラメータ
	vector<cv::Scalar> lineColor,				//直線の色
	makeProjImg adjusted						//糸全体調整時のパラメータ
) {
	//返り値を初期化
	makeProjImg ret = adjusted;

	//糸の上端下端座標をスケールするためのvector
	std::vector<cv::Point3d> thr_pos_scaled;
	thr_pos_scaled = thr_pos;

	while (1) {
		int selectedThread;
		cout << "糸のインデックス(0 ~ " << thr_pos.size() / 2 - 1 << ")を入力してください、-1で調整完了：";
		cin >> selectedThread;

		// int 以外が入力された場合
		if (std::cin.fail()) {
			cin.clear(); // fail フラグをクリア
			cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

			cerr << "入力がint型ではありません" << endl;
			continue;
		}
		//調整を終了する条件、-1を入力すると調整終了
		else if (selectedThread == -1) {
			break;
		}
		//糸のインデックスが要素数内じゃない場合
		else if (selectedThread < 0 || thr_pos.size() / 2 - 1 < selectedThread) {
			cout << "糸のインデックスが不正です" << endl;
			continue;
		}

		selectedThread *= 2;	//糸座標のインデックスで上端のみを指定するために2倍

		//vdの糸配置を画像で表示、調整対象の糸をハイライト
		cv::Mat threadArrengment = makeImg_threadArrengiment(thr_pos, selectedThread);

		//スライダーは整数なのでスケールして扱う（0.01単位）
		auto s2dbl = [](int v) { return v / SLIDER_SCALE; };

		//トラックバー関連の変数
		int tx_s = 0, ty_s = 0, tz_s = 0;
		int roll_s = 0, pitch_s = 0, yaw_s = 0;
		const int SminShift = -200, SmaxShift = 200;	//並進移動トラックバーの上限下限
		const int SminRot = -400, SmaxRot = 400;		//回転移動トラックバーの上限下限

		//直線の太さ
		int width_s = adjusted.lineWidth;
		const int SminWidth = 1, SmaxWidth = 256;

		//直線の長さ
		int length_s = adjusted.lineLength;
		const int SminLength = 1, SmaxLength = 100;

		//背景色
		int back_s = 0;
		const int SminBack = 0, SmaxBack = 255;

		//スライダーのウィンドウを作成
		cv::namedWindow("Tune Projection", cv::WINDOW_NORMAL);
		cv::resizeWindow("Tune Projection", 1280, 800);
		cv::createTrackbar("tx ", "Tune Projection", &tx_s, SmaxShift); cv::setTrackbarMin("tx ", "Tune Projection", SminShift);
		cv::createTrackbar("ty ", "Tune Projection", &ty_s, SmaxShift); cv::setTrackbarMin("ty ", "Tune Projection", SminShift);
		cv::createTrackbar("tz ", "Tune Projection", &tz_s, SmaxShift); cv::setTrackbarMin("tz ", "Tune Projection", SminShift);
		cv::createTrackbar("roll ", "Tune Projection", &roll_s, SmaxRot); cv::setTrackbarMin("roll ", "Tune Projection", SminRot);
		cv::createTrackbar("pitch ", "Tune Projection", &pitch_s, SmaxRot); cv::setTrackbarMin("pitch ", "Tune Projection", SminRot);
		cv::createTrackbar("yaw ", "Tune Projection", &yaw_s, SmaxRot); cv::setTrackbarMin("yaw ", "Tune Projection", SminRot);
		cv::createTrackbar("width ", "Tune Projection", &width_s, SmaxWidth); cv::setTrackbarMin("width ", "Tune Projection", SminWidth);
		cv::createTrackbar("length ", "Tune Projection", &length_s, SmaxLength); cv::setTrackbarMin("length ", "Tune Projection", SminLength);
		cv::createTrackbar("backColor ", "Tune Projection", &back_s, SmaxBack); cv::setTrackbarMin("backColor ", "Tune Projection", SminBack);

		cout << endl;
		cout << "コマンドリスト" << endl;
		cout << "Enter	プログラム終了" << endl;
		cout << "s	現在の糸座標を保存" << endl;
		cout << "r	補正をリセット" << endl;
		cout << endl;

		while (true) {

			// 現在のΔを実数化
			const double tx = s2dbl(tx_s);
			const double ty = s2dbl(ty_s);
			const double tz = s2dbl(tz_s);
			const double rX = s2dbl(roll_s);
			const double rY = s2dbl(pitch_s);
			const double rZ = s2dbl(yaw_s);
			const int wd = width_s;
			const double ln = length_s / 100.0;
			const int bc = back_s;

			// ΔR を左乗、Δt を加算
			cv::Mat R_delta = so3Exp_deg(rX, rY, rZ);
			cv::Mat R_adj = R_delta * adjusted.R_adj;
			cv::Mat t_adj = adjusted.t_adj.clone();
			t_adj.at<double>(0) += tx;
			t_adj.at<double>(1) += ty;
			t_adj.at<double>(2) += tz;

			//糸の3次元座標を糸の中心方向にスケーリング
			//thr_pos_scaled.reserve(thr_pos.size());

			cv::Point3d thr_top = thr_pos[selectedThread];
			cv::Point3d thr_bottom = thr_pos[selectedThread + 1];
			cv::Point3d thr_center = (thr_top + thr_bottom) * 0.5;	//糸の中心を表す3次元座標
			cv::Point3d d = thr_top - thr_bottom;					//糸の下端から上端を表すベクトル

			cv::Point3d thr_top_s = thr_center + 0.5 * ln * d;
			cv::Point3d thr_bottom_s = thr_center - 0.5 * ln * d;

			thr_pos_scaled[selectedThread] = thr_top_s;
			thr_pos_scaled[selectedThread + 1] = thr_bottom_s;


			// 調整後の投影画像
			//makeProjImg resMeasure = make_projectionImg(thr_pos, R_adj, t_adj, backColor, lineColor, lineWidth);	//yawのみ対応
			makeProjImg resMeasure = make_projectionImg_projective_individually(
				thr_pos_scaled, adjusted.R_adj, adjusted.t_adj, projP,
				cv::Size(PROJ_IMG_WIDTH, PROJ_IMG_HEIGHT),
				cv::Scalar(bc, bc, bc),
				lineColor, wd,
				selectedThread, R_adj, t_adj, adjusted.thr_pos_projImg);

			// テキストオーバーレイ
			cv::Mat disp = resMeasure.projImg.clone();
			char buf[256];
			std::snprintf(buf, sizeof(buf),
				"(x, y, z)[cm]=[%.2f, %.2f, %.2f]", tx, ty, tz);
			cv::putText(disp, buf, { 20, 40 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 40,40,40 }, 3, cv::LINE_AA);
			cv::putText(disp, buf, { 20, 40 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 240,240,240 }, 1, cv::LINE_AA);

			char buf2[256];
			std::snprintf(buf2, sizeof(buf2),
				"(roll, pitch, yaw)[deg]=[%.2f, %.2f, %.2f]", rX, rY, rZ);
			cv::putText(disp, buf2, { 20, 90 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 40,40,40 }, 3, cv::LINE_AA);
			cv::putText(disp, buf2, { 20, 90 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 240,240,240 }, 1, cv::LINE_AA);

			char buf3[256];
			std::snprintf(buf3, sizeof(buf3),
				"line width[px]=%d  length[%%]=%d", wd, length_s);
			cv::putText(disp, buf3, { 20, 140 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 40,40,40 }, 3, cv::LINE_AA);
			cv::putText(disp, buf3, { 20, 140 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 240,240,240 }, 1, cv::LINE_AA);

			char buf4[256];
			std::snprintf(buf4, sizeof(buf4),
				"backColor=(%d, %d, %d)", bc, bc, bc);
			cv::putText(disp, buf4, { 20, 190 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 40,40,40 }, 3, cv::LINE_AA);
			cv::putText(disp, buf4, { 20, 190 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 240,240,240 }, 1, cv::LINE_AA);

			//画像を表示
			cv::imshow("Tune Projection", disp);
			cv::imshow("selectedThread", threadArrengment);

			cv::namedWindow("Tuned img", cv::WINDOW_NORMAL);
			cv::setWindowProperty("Tuned img", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
			cv::moveWindow("Tuned img", 1920, -100);
			cv::imshow("Tuned img", resMeasure.projImg);

			//コマンド
			int key = cv::waitKey(30);
			if (key == 13) {	// Enterで閉じる
				ret.lineWidth = 0;

				cv::Mat projImgTmp(cv::Size(PROJ_IMG_WIDTH, PROJ_IMG_HEIGHT), CV_8UC3, cv::Scalar(0, 0, 0));
				ret.projImg = projImgTmp;

				vector<cv::Point2i> thr_pos_projImg_tmp;
				ret.thr_pos_projImg = thr_pos_projImg_tmp;

				cout << "補正情報を保存せずに終了" << endl;
				cout << endl;

				break;
			}
			if (key == 's') {	// sで現在の状態を保存し出力
				//背景を黒にして，投影画像を作成
				ret = make_projectionImg_projective_individually(
					thr_pos_scaled, adjusted.R_adj, adjusted.t_adj, projP,
					cv::Size(PROJ_IMG_WIDTH, PROJ_IMG_HEIGHT),
					cv::Scalar(bc, bc, bc),
					lineColor, wd,
					selectedThread, R_adj, t_adj, adjusted.thr_pos_projImg);

				//調整後の回転・並進ベクトル、投影画像中の糸座標、光線の太さを保存
				ret.R_adj = R_adj;
				ret.t_adj = t_adj;
				adjusted.thr_pos_projImg = ret.thr_pos_projImg;
				adjusted.lineWidth = ret.lineWidth;

				cout << endl;

				cv::imshow("Tuned img", ret.projImg);
				cv::imwrite("projImg_threads_tuned.bmp", ret.projImg);

				break;
			}
			if (key == 'r') {	// rで全リセット（スライダーと糸座標）
				//並進ベクトル・回転行列をリセット
				R_adj = adjusted.R_adj;
				t_adj = adjusted.t_adj;

				// トラックバー位置を中央（0）に戻す
				tx_s = ty_s = tz_s = roll_s = pitch_s = yaw_s = 0;
				width_s = adjusted.lineWidth;
				length_s = adjusted.lineLength;
				back_s = 0;
				cv::setTrackbarPos("tx ", "Tune Projection", tx_s);
				cv::setTrackbarPos("ty ", "Tune Projection", ty_s);
				cv::setTrackbarPos("tz ", "Tune Projection", tz_s);
				cv::setTrackbarPos("roll ", "Tune Projection", roll_s);
				cv::setTrackbarPos("pitch ", "Tune Projection", pitch_s);
				cv::setTrackbarPos("yaw ", "Tune Projection", yaw_s);
				cv::setTrackbarPos("width ", "Tune Projection", width_s);
				cv::setTrackbarPos("length ", "Tune Projection", length_s);
				cv::setTrackbarPos("backColor ", "Tune Projection", back_s);

				cout << "補正をリセット" << endl;
			}
		}
		cv::destroyWindow("Tune Projection");
	}

	return ret;
}



//-----調整後のパラメータから作成済み指向性画像を補正-----------------------------------------
//元の指向性画像を補正し，投影部分が重なった時に光線を削除する関数
void make_directionalImg_filtered(
	const cv::Mat& directionalImg,	//元の指向性画像（未補正）
	makeProjImg mpi					//補正後の糸座標データ
) {
	//---エラーチェック--------------------------------------------------
	makeProjImg result = mpi;	//補正結果をコピーして編集

	//resultに返り値が入っているかをチェック
	if (result.lineWidth <= 0 || result.projImg.empty() || result.thr_pos_projImg.empty()) {
		cout << "makeProjImgのメンバ変数がnullです" << endl;
		return;
	}

	const auto& coords = mpi.thr_pos_projImg;
	const int lineWidth = mpi.lineWidth;

	//補正済み投影平面上の糸座標が上端下端ペアで保存されているかチェック
	if (coords.size() % 2 != 0) {
		cout << "補正済み投影平面上の糸座標のサイズが奇数です" << endl;
		return;
	}

	const int numThreads = coords.size() / 2;	//投影範囲内の糸の本数

	//---投影画像の初期化------------------------------------------------
	//出力画像を定義
	//一度scale倍された画像で描画し，後で縮小することでアンチエイリアスを行う
	const double scale = 3.0;
	cv::Mat canvas(				//縮小前
		result.projImg.rows * scale,
		result.projImg.cols * scale,
		CV_8UC3,
		cv::Scalar(0, 0, 0)
	);

	cv::Mat finalCanvas;		//縮小後

	//プロジェクターと作成済みの指向性画像の解像度が一致するかチェック
	if (canvas.cols / (int)scale != directionalImg.cols || canvas.rows / (int)scale != directionalImg.rows) {
		cout << "プロジェクターと作成済みの指向性画像の解像度が一致しません" << endl;
		return;
	}

	//---各糸の描画範囲を事前計算-----------------------------------------------
	vector<vector<cv::Point2f>> polygons(numThreads);			//重なり判定前の描画範囲を保存
	const int sliceWidth = directionalImg.cols / numThreads;	//糸1本あたりの矩形画像の幅
	const int sliceHeight = directionalImg.rows;				//糸1本あたりの矩形画像の高さ

	for (int i = 0; i < numThreads; i++) {

		//糸の上端・下端座標を取得，scaleに合わせて座標を移動
		cv::Point2f top(coords[2 * i].x * scale, coords[2 * i].y * scale);
		cv::Point2f bottom(coords[2 * i + 1].x * scale, coords[2 * i + 1].y * scale);

		//上端から下端のベクトルを計算
		cv::Point2f direction(bottom - top);
		float length = cv::norm(direction);
		if (length < 1) continue;	//ベクトルの長さが1未満の時にスキップ
		direction /= length;		//ベクトルの長さで割って，単位ベクトルに変換

		//幅方向（法線）ベクトルを計算
		cv::Point2f normal(-direction.y, direction.x);
		normal *= (lineWidth * scale) * 0.5f;

		//描画範囲（糸部分）の四角形を定義
		//左上から時計周りに定義
		polygons[i] = {
			top + normal,
			top - normal,
			bottom - normal,
			bottom + normal
		};
	}

	//---重なり解析，全ペア比較し，重なった糸は両方除外--------------------
	std::vector<bool> isValid(numThreads, true);	//各糸が有効かどうかのフラグを保存
	const double overlapThreshold = 1.0;			//重なった面積の閾値

	//糸の全ペアで重なり判定
	for (int i = 0; i < numThreads; i++) {
		if (polygons[i].empty()) continue;

		for (int j = i + 1; j < numThreads; j++) {
			if (polygons[j].empty()) continue;

			std::vector<cv::Point2f> inter;	//重なった領域の多角形座標

			//投影範囲同士の重なっている面積を計算
			double area = cv::intersectConvexConvex(polygons[i], polygons[j], inter);

			//重なった面積が閾値より大きい場合
			if (area > overlapThreshold) {

				//i, j番目の糸を両方無効化
				isValid[i] = false;
				isValid[j] = false;
			}
		}
	}

	//---warpPerspectiveで有効糸のみ描画------------------------------------
	//作成済み指向性画像のスライスの四角形座標
	std::vector<cv::Point2f> srcQuad{
		{0,0},
		{(float)sliceWidth, 0},
		{(float)sliceWidth, (float)sliceHeight},
		{0, (float)sliceHeight}
	};

	//有効な糸をカウント
	int validCount = 0;

	for (int i = 0; i < numThreads; i++) {

		if (!isValid[i]) continue;	//無効な糸だったらスキップ

		//元指向性画像から，糸に対応するスライスを切り抜く
		cv::Rect sliceROI(i * sliceWidth, 0, sliceWidth, sliceHeight);
		cv::Mat slice = directionalImg(sliceROI);

		//srcQuadからdstQuadへの変換行列
		cv::Mat H = cv::getPerspectiveTransform(srcQuad, polygons[i]);

		//変換行列に基づいて，スライスを貼り付け
		cv::warpPerspective(
			slice,
			canvas,
			H,
			canvas.size(),
			cv::INTER_LINEAR,
			cv::BORDER_TRANSPARENT
		);

		validCount++;
	}

	//補正済み指向性画像を縮小
	cv::resize(canvas, finalCanvas, result.projImg.size(), 0, 0, cv::INTER_AREA);

	cout << "使用された糸の本数： " << validCount << " / " << numThreads << endl;

	//---結果表示-----------------------------------------------------------
	//投影画像のwindowをフルスクリーン化
	cv::namedWindow("Filtered Directional Img", cv::WINDOW_NORMAL);
	cv::setWindowProperty("Filtered Directional Img", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
	cv::moveWindow("Filtered Directional Img", 1920, -100);
	cv::imshow("Filtered Directional Img", finalCanvas);

	cv::imwrite("projImg_threads_directional_filtered.bmp", finalCanvas);
	cv::waitKey(0);
}





//-----main関数-----------------------------------------------------------------
int main() {

	//糸のユークリッド座標を保存するlist, vdの左下が原点
	vector<cv::Point3d> thr_pos_original = setThreadsCoordinate64();

	//糸の端の座標を追加
	vector<cv::Point3d> thr_pos_ends = setThreadEndsCoordinate(thr_pos_original);

	//糸の座標を加工
	//arMarkerをボリュームディスプレイの上部に付けるとして加工
	vector<cv::Point3d> thr_pos = changeThreadsCoordinate(thr_pos_ends);

	//カメラの内部パラメーターを取得
	cout << "カメラの内部パラメータを取得" << endl;
	parameters_cc p_camMat = getCcParameters();	//下1桁が四捨五入されている

	//使用したArUcoマーカーの設定
	cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);	//DICT_(bit数)_(arMarkerの作成可能枚数)

	//arMarkerの検出開始
	cout << "ArUcoMarkerを検出" << endl;

	//静止画像からArUcoマーカーを検出
	result_test_arMarkerDetect out = test_arMarkerDetect(dictionary, p_camMat, "testImg.bmp");

	//投影画像を微調整
	//糸全体を動かして微調整
	parameters_cc projP = build_proj_inParams_FromAOV(AOV_X, cv::Size(PROJ_IMG_WIDTH, PROJ_IMG_HEIGHT));	//projの内部パラメータを仮で設定
	vector<cv::Scalar> lineColor = {	//直線の色を定義
		cv::Scalar(60, 60, 255),		//赤
		cv::Scalar(60, 255, 60),		//緑
		cv::Scalar(255, 60, 60),		//青
		cv::Scalar(255, 255, 60),		//シアン
		cv::Scalar(255, 60, 255),		//マゼンタ
		cv::Scalar(60, 255, 255),		//イエロー
		cv::Scalar(0, 165, 255),		//オレンジ
		cv::Scalar(180, 180, 255)		//薄いピンク
	};
	makeProjImg adjusted = interactiveAdjustProjection(thr_pos, out.R_relative, out.t_relative, projP, lineColor);	//微調整UI

	//糸1本1本を動かして微調整
	vector<cv::Scalar> lineColor_individually = {	//直線の色を定義
		cv::Scalar(60, 60, 255),		//赤
		cv::Scalar(255, 60, 60),		//青
	};
	makeProjImg adjusted2 = interactiveAdjustProjection_individually(thr_pos, projP, lineColor_individually, adjusted);	//微調整UI

	//作成済みの指向性画像を補正
	cout << "作成済みの指向性画像を元に，補正後画像を作成" << endl;
	cv::Mat dImg = cv::imread("projImg_threads.bmp");
	make_directionalImg_filtered(dImg, adjusted2);

	return 0;
}