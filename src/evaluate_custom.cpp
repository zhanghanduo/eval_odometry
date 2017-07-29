//
// Created by zh on 12/7/17.
//
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <limits>
#include <unistd.h>
#include <string.h>
#include "mail.h"
#include "matrix.h"
#include <map>
#include <gflags/gflags.h>

using namespace std;

// static parameter
DEFINE_int32(num, 1, "number of evaluation");
DEFINE_string(prefix, "test", "name prefix of groundtruth and slam pose file");
DEFINE_double(offset, 0, "offset during alignment of two files");
DEFINE_double(maxdiff, 0.02, "maximum value difference between aligned frames");

// float lengths[] = {5,10,50,100,150,200,250,300,350,400};
float lengths[] = {100,200,300,400,500,600,700,800};
int32_t num_lengths = 8;

typedef map<double, Matrix> posemap;
typedef vector<pair<Matrix, Matrix>> posepair;
vector<Matrix> poses_gt, poses_result;

std::string getexepath()
{
    char result[ 256 ];
    ssize_t count = readlink( "/proc/self/exe", result, 256 );
    return std::string( result, (count > 0) ? count : 0 );
}

void associate(posemap gt, posemap est, double offset, double max_difference){
//    posepair matches;
    poses_gt.clear();
    poses_result.clear();
    map<double, array<Matrix, 2>> potential_matches;
//    vector<pair<double, array<Matrix,2>>> potential_matches;

    for (auto git = gt.begin(); git != gt.end(); ++git){
        for (auto eit = est.begin(); eit != est.end(); ++eit){
            double value;
            value = fabs(git->first - (eit->first + offset));
            if(value < max_difference) {
//                array<Matrix,2> tempMatrix_array;
//                tempMatrix_array[0] = git->second;
//                tempMatrix_array[1] = eit->second;
//                potential_matches.push_back(make_pair(value, tempMatrix_array));
                potential_matches[value] = {git->second, eit->second};
            }
        }
    }
//        std::sort(potential_matches.begin(), potential_matches.end()); // no need to sort map by its key
    for (auto it = potential_matches.begin(); it != potential_matches.end(); ++it){
//        matches.insert(make_pair(it->second[0],it->second[1]));
        poses_gt.push_back(it->second[0]);
        poses_result.push_back(it->second[1]);
//        matches.emplace_back(it->second[0], it->second[1]);
    }
}

struct errors {
    int32_t first_frame;
    float   r_err;
    float   t_err;
    float   len;
    float   speed;
    errors (int32_t first_frame,float r_err,float t_err,float len,float speed) :
            first_frame(first_frame),r_err(r_err),t_err(t_err),len(len),speed(speed) {}
};


posemap loadPoses_map(string file_name){
    posemap poses;
    FILE *fp = fopen(file_name.c_str(),"r");
    if (fp == nullptr)
        return poses;
    while (feof(fp) == 0) {
        Matrix P = Matrix::eye(4);
        double t;
        if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &t,
                   &P.val[0][0], &P.val[0][1], &P.val[0][2], &P.val[0][3],
                   &P.val[1][0], &P.val[1][1], &P.val[1][2], &P.val[1][3],
                   &P.val[2][0], &P.val[2][1], &P.val[2][2], &P.val[2][3] )==13) {
            poses[t] = P;
        }
    }
    fclose(fp);
    return poses;
};


vector<Matrix> loadPoses(string file_name) {
    vector<Matrix> poses;
    FILE *fp = fopen(file_name.c_str(),"r");
    if (!fp)
        return poses;
    while (!feof(fp)) {
        Matrix P = Matrix::eye(4);
        if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   &P.val[0][0], &P.val[0][1], &P.val[0][2], &P.val[0][3],
                   &P.val[1][0], &P.val[1][1], &P.val[1][2], &P.val[1][3],
                   &P.val[2][0], &P.val[2][1], &P.val[2][2], &P.val[2][3] )==12) {
            poses.push_back(P);
        }
    }
    fclose(fp);
    return poses;
}

vector<float> trajectoryDistances_t (posepair &poses) {
    vector<float> dist;
    dist.push_back(0);
    for (int32_t i=1; i<poses.size(); i++) {
        Matrix P1 = poses[i-1].first;
        Matrix P2 = poses[i].first;
        double dx = P1.val[0][3]-P2.val[0][3];
        double dy = P1.val[1][3]-P2.val[1][3];
        double dz = P1.val[2][3]-P2.val[2][3];
        dist.push_back(dist[i-1]+sqrt(dx*dx+dy*dy+dz*dz));
    }
    return dist;
}


vector<float> trajectoryDistances (vector<Matrix> &poses) {
    vector<float> dist;
    dist.push_back(0);
    for (int32_t i=1; i<poses.size(); i++) {
        Matrix P1 = poses[i-1];
        Matrix P2 = poses[i];
        float dx = P1.val[0][3]-P2.val[0][3];
        float dy = P1.val[1][3]-P2.val[1][3];
        float dz = P1.val[2][3]-P2.val[2][3];
        dist.push_back(dist[i-1]+sqrt(dx*dx+dy*dy+dz*dz));
    }
    return dist;
}

int32_t lastFrameFromSegmentLength(vector<float> &dist,int32_t first_frame,float len) {
    for (int32_t i=first_frame; i<dist.size(); i++)
        if (dist[i]>dist[first_frame]+len)
            return i;
    return -1;
}

inline float rotationError(Matrix &pose_error) {
    float a = pose_error.val[0][0];
    float b = pose_error.val[1][1];
    float c = pose_error.val[2][2];
    float d = 0.5*(a+b+c-1.0);
    return acos(max(min(d,1.0f),-1.0f));
}

inline float translationError(Matrix &pose_error) {
    float dx = pose_error.val[0][3];
    float dy = pose_error.val[1][3];
    float dz = pose_error.val[2][3];
    return sqrt(dx*dx+dy*dy+dz*dz);
}

vector<errors> calSequenceError_t (posepair &pose_pairs) {

    // error vector
    vector<errors> err;

    // parameters
    int32_t step_size = 10; // every second

    // pre-compute distances (from ground truth as reference)
    vector<float> dist = trajectoryDistances_t(pose_pairs);

    // for all start positions do
    for (int32_t first_frame=0; first_frame<pose_pairs.size(); first_frame+=step_size) {

        // for all segment lengths do
        for (int32_t i=0; i<num_lengths; i++) {

            // current length
            float len = lengths[i];

            // compute last frame
            int32_t last_frame = lastFrameFromSegmentLength(dist,first_frame,len);

            // continue, if sequence not long enough
            if (last_frame==-1)
                continue;

            // compute rotational and translational errors
            Matrix pose_delta_gt     = Matrix::inv(pose_pairs[first_frame].first)*pose_pairs[last_frame].first;
            Matrix pose_delta_result = Matrix::inv(pose_pairs[first_frame].second)*pose_pairs[last_frame].second;
            Matrix pose_error        = Matrix::inv(pose_delta_result)*pose_delta_gt;
            float r_err = rotationError(pose_error);
            float t_err = translationError(pose_error);

            // compute speed
            auto num_frames = (float)(last_frame-first_frame+1);
            float speed = len/(0.1*num_frames);

            // write to file
            err.push_back(errors(first_frame,r_err/len,t_err/len,len,speed));
        }
    }

    // return error vector
    return err;
}

vector<errors> calcSequenceErrors (vector<Matrix> &poses_gt,vector<Matrix> &poses_result) {

    // error vector
    vector<errors> err;

    // parameters
    int32_t step_size = 10; // every second

    // pre-compute distances (from ground truth as reference)
    vector<float> dist = trajectoryDistances(poses_gt);

    // for all start positions do
    for (int32_t first_frame=0; first_frame<poses_gt.size(); first_frame+=step_size) {

        // for all segment lengths do
        for (int32_t i=0; i<num_lengths; i++) {

            // current length
            float len = lengths[i];

            // compute last frame
            int32_t last_frame = lastFrameFromSegmentLength(dist,first_frame,len);

            // continue, if sequence not long enough
            if (last_frame==-1)
                continue;

            // compute rotational and translational errors
            Matrix pose_delta_gt     = Matrix::inv(poses_gt[first_frame])*poses_gt[last_frame];
            Matrix pose_delta_result = Matrix::inv(poses_result[first_frame])*poses_result[last_frame];
            Matrix pose_error        = Matrix::inv(pose_delta_result)*pose_delta_gt;
            float r_err = rotationError(pose_error);
            float t_err = translationError(pose_error);

            // compute speed
            auto num_frames = (float)(last_frame-first_frame+1);
            float speed = len/(0.1*num_frames);

            // write to file
            err.push_back(errors(first_frame,r_err/len,t_err/len,len,speed));
        }
    }

    // return error vector
    return err;
}

void saveSequenceErrors (vector<errors> &err,string file_name) {

    // open file
    FILE *fp;
    fp = fopen(file_name.c_str(),"w");

    // write to file
    for (vector<errors>::iterator it=err.begin(); it!=err.end(); it++)
        fprintf(fp,"%d %f %f %f %f\n",it->first_frame,it->r_err,it->t_err,it->len,it->speed);

    // close file
    fclose(fp);
}

void savePathPlot (vector<Matrix> &poses_gt,vector<Matrix> &poses_result,string file_name) {

    // parameters
    int32_t step_size = 3;

    // open file
    FILE *fp = fopen(file_name.c_str(),"w");

    // save x/z coordinates of all frames to file
    for (int32_t i=0; i<poses_gt.size(); i+=step_size)
        fprintf(fp,"%f %f %f %f\n",poses_gt[i].val[0][3],poses_gt[i].val[2][3],
                poses_result[i].val[0][3],poses_result[i].val[2][3]);

    // close file
    fclose(fp);
}

vector<int32_t> computeRoi (vector<Matrix> &poses_gt,vector<Matrix> &poses_result) {

    float x_min = numeric_limits<int32_t>::max();
    float x_max = numeric_limits<int32_t>::min();
    float z_min = numeric_limits<int32_t>::max();
    float z_max = numeric_limits<int32_t>::min();

    for (vector<Matrix>::iterator it=poses_gt.begin(); it!=poses_gt.end(); it++) {
        float x = it->val[0][3];
        float z = it->val[2][3];
        if (x<x_min) x_min = x; if (x>x_max) x_max = x;
        if (z<z_min) z_min = z; if (z>z_max) z_max = z;
    }

    for (vector<Matrix>::iterator it=poses_result.begin(); it!=poses_result.end(); it++) {
        float x = it->val[0][3];
        float z = it->val[2][3];
        if (x<x_min) x_min = x; if (x>x_max) x_max = x;
        if (z<z_min) z_min = z; if (z>z_max) z_max = z;
    }

    float dx = 1.1*(x_max-x_min);
    float dz = 1.1*(z_max-z_min);
    float mx = 0.5*(x_max+x_min);
    float mz = 0.5*(z_max+z_min);
    float r  = 0.5*max(dx,dz);

    vector<int32_t> roi;
    roi.push_back((int32_t)(mx-r));
    roi.push_back((int32_t)(mx+r));
    roi.push_back((int32_t)(mz-r));
    roi.push_back((int32_t)(mz+r));
    return roi;
}

void plotPathPlot (string dir,vector<int32_t> &roi,int32_t idx) {

    // gnuplot file name
    char command[1024];
    char file_name[256];
    sprintf(file_name,"%02d.gp",idx);
    string full_name = dir + "/" + file_name;

    // create png + eps
    for (int32_t i=0; i<2; i++) {

        // open file
        FILE *fp = fopen(full_name.c_str(),"w");

        // save gnuplot instructions
        if (i==0) {
            fprintf(fp,"set term png size 900,900\n");
            fprintf(fp,"set output \"%02d.png\"\n",idx);
        } else {
            fprintf(fp,"set term postscript eps enhanced color\n");
            fprintf(fp,"set output \"%02d.eps\"\n",idx);
        }

        fprintf(fp,"set size ratio -1\n");
        fprintf(fp,"set xrange [%d:%d]\n",roi[0],roi[1]);
        fprintf(fp,"set yrange [%d:%d]\n",roi[2],roi[3]);
        fprintf(fp,"set xlabel \"x [m]\"\n");
        fprintf(fp,"set ylabel \"z [m]\"\n");
        fprintf(fp,"plot \"%02d.txt\" using 1:2 lc rgb \"#FF0000\" title 'Ground Truth' w lines,",idx);
        fprintf(fp,"\"%02d.txt\" using 3:4 lc rgb \"#0000FF\" title 'Visual Odometry' w lines,",idx);
        fprintf(fp,"\"< head -1 %02d.txt\" using 1:2 lc rgb \"#000000\" pt 4 ps 1 lw 2 title 'Sequence Start' w points\n",idx);

        // close file
        fclose(fp);

        // run gnuplot => create png + eps
        sprintf(command,"cd %s; gnuplot %s",dir.c_str(),file_name);
        system(command);
    }

    // create pdf and crop
    sprintf(command,"cd %s; ps2pdf %02d.eps %02d_large.pdf",dir.c_str(),idx,idx);
    system(command);
    sprintf(command,"cd %s; pdfcrop %02d_large.pdf %02d.pdf",dir.c_str(),idx,idx);
    system(command);
    sprintf(command,"cd %s; rm %02d_large.pdf",dir.c_str(),idx);
    system(command);
}

void saveErrorPlots(vector<errors> &seq_err,string plot_error_dir,char* prefix) {

    // file names
    char file_name_tl[1024]; sprintf(file_name_tl,"%s/%s_tl.txt",plot_error_dir.c_str(),prefix);
    char file_name_rl[1024]; sprintf(file_name_rl,"%s/%s_rl.txt",plot_error_dir.c_str(),prefix);
    char file_name_ts[1024]; sprintf(file_name_ts,"%s/%s_ts.txt",plot_error_dir.c_str(),prefix);
    char file_name_rs[1024]; sprintf(file_name_rs,"%s/%s_rs.txt",plot_error_dir.c_str(),prefix);

    // open files
    FILE *fp_tl = fopen(file_name_tl,"w");
    FILE *fp_rl = fopen(file_name_rl,"w");
    FILE *fp_ts = fopen(file_name_ts,"w");
    FILE *fp_rs = fopen(file_name_rs,"w");

    // for each segment length do
    for (int32_t i=0; i<num_lengths; i++) {

        float t_err = 0;
        float r_err = 0;
        float num   = 0;

        // for all errors do
        for (vector<errors>::iterator it=seq_err.begin(); it!=seq_err.end(); it++) {
            if (fabs(it->len-lengths[i])<1.0) {
                t_err += it->t_err;
                r_err += it->r_err;
                num++;
            }
        }

        // we require at least 3 values
        if (num>2.5) {
            fprintf(fp_tl,"%f %f\n",lengths[i],t_err/num);
            fprintf(fp_rl,"%f %f\n",lengths[i],r_err/num);
        }
    }

    // for each driving speed do (in m/s)
    for (float speed=2; speed<25; speed+=2) {

        float t_err = 0;
        float r_err = 0;
        float num   = 0;

        // for all errors do
        for (vector<errors>::iterator it=seq_err.begin(); it!=seq_err.end(); it++) {
            if (fabs(it->speed-speed)<2.0) {
                t_err += it->t_err;
                r_err += it->r_err;
                num++;
            }
        }

        // we require at least 3 values
        if (num>2.5) {
            fprintf(fp_ts,"%f %f\n",speed,t_err/num);
            fprintf(fp_rs,"%f %f\n",speed,r_err/num);
        }
    }

    // close files
    fclose(fp_tl);
    fclose(fp_rl);
    fclose(fp_ts);
    fclose(fp_rs);
}

void plotErrorPlots (string dir,char* prefix) {

    char command[1024];

    // for all four error plots do
    for (int32_t i=0; i<4; i++) {

        // create suffix
        char suffix[16];
        switch (i) {
            case 0: sprintf(suffix,"tl"); break;
            case 1: sprintf(suffix,"rl"); break;
            case 2: sprintf(suffix,"ts"); break;
            case 3: sprintf(suffix,"rs"); break;
        }

        // gnuplot file name
        char file_name[1024]; char full_name[1024];
        sprintf(file_name,"%s_%s.gp",prefix,suffix);
        sprintf(full_name,"%s/%s",dir.c_str(),file_name);

        // create png + eps
        for (int32_t j=0; j<2; j++) {

            // open file
            FILE *fp = fopen(full_name,"w");

            // save gnuplot instructions
            if (j==0) {
                fprintf(fp,"set term png size 500,250 font \"Helvetica\" 11\n");
                fprintf(fp,"set output \"%s_%s.png\"\n",prefix,suffix);
            } else {
                fprintf(fp,"set term postscript eps enhanced color\n");
                fprintf(fp,"set output \"%s_%s.eps\"\n",prefix,suffix);
            }

            // start plot at 0
            fprintf(fp,"set size ratio 0.5\n");
            fprintf(fp,"set yrange [0:*]\n");

            // x label
            if (i<=1) fprintf(fp,"set xlabel \"Path Length [m]\"\n");
            else      fprintf(fp,"set xlabel \"Speed [km/h]\"\n");

            // y label
            if (i==0 || i==2) fprintf(fp,"set ylabel \"Translation Error [%%]\"\n");
            else              fprintf(fp,"set ylabel \"Rotation Error [deg/m]\"\n");

            // plot error curve
            fprintf(fp,"plot \"%s_%s.txt\" using ",prefix,suffix);
            switch (i) {
                case 0: fprintf(fp,"1:($2*100) title 'Translation Error'"); break;
                case 1: fprintf(fp,"1:($2*57.3) title 'Rotation Error'"); break;
                case 2: fprintf(fp,"($1*3.6):($2*100) title 'Translation Error'"); break;
                case 3: fprintf(fp,"($1*3.6):($2*57.3) title 'Rotation Error'"); break;
            }
            fprintf(fp," lc rgb \"#0000FF\" pt 4 w linespoints\n");

            // close file
            fclose(fp);

            // run gnuplot => create png + eps
            sprintf(command,"cd %s; gnuplot %s",dir.c_str(),file_name);
            system(command);
        }

        // create pdf and crop
        sprintf(command,"cd %s; ps2pdf %s_%s.eps %s_%s_large.pdf",dir.c_str(),prefix,suffix,prefix,suffix);
        system(command);
        sprintf(command,"cd %s; pdfcrop %s_%s_large.pdf %s_%s.pdf",dir.c_str(),prefix,suffix,prefix,suffix);
        system(command);
        sprintf(command,"cd %s; rm %s_%s_large.pdf",dir.c_str(),prefix,suffix);
        system(command);
    }
}

void saveStats (vector<errors> err,string dir) {

    float t_err = 0;
    float r_err = 0;

    // for all errors do => compute sum of t_err, r_err
    for (vector<errors>::iterator it=err.begin(); it!=err.end(); it++) {
        t_err += it->t_err;
        r_err += it->r_err;
    }

    // open file
    FILE *fp = fopen((dir + "/stats.txt").c_str(),"w");

    // save errors
    float num = err.size();
    fprintf(fp,"%f %f\n",t_err/num,r_err/num);

    // close file
    fclose(fp);
}

bool eval (Mail* mail) {

    string work_dir_build = getexepath();
    string work_dir = work_dir_build.substr(0, work_dir_build.find("cmake-build"));
    cout << "current working dir is: " << work_dir << endl;
    // ground truth and result directories
    string gt_dir         = work_dir + "eval_data/gt";  //ground truth trajectory file, named 01.txt, 02.txt ...
    string result_dir     = work_dir + "eval_data/results/";   //results.txt
    string error_dir      = result_dir + "/errors";
    string plot_path_dir  = result_dir + "/plot_path";
    string plot_error_dir = result_dir + "/plot_error";

    // create output directories
    system(("mkdir " + error_dir).c_str());
    system(("mkdir " + plot_path_dir).c_str());
    system(("mkdir " + plot_error_dir).c_str());

    // total errors
    vector<errors> total_err;

    // for all sequences do
    for (int32_t i = 1; i < FLAGS_num + 1; i++) {

        // file name
        char file_name[64];
        sprintf(file_name,"%s%02d.txt",FLAGS_prefix.c_str(),i);

        // read ground truth and result poses
//        vector<Matrix> poses_gt_p     = loadPoses(gt_dir + "/" + file_name);
//        vector<Matrix> poses_result_p = loadPoses(result_dir + "/data/" + file_name);

        posemap poses_gt_p = loadPoses_map(gt_dir + "/" + file_name);
        std::cout << "Loaded gt result poses" << std::endl;

        posemap poses_result_p = loadPoses_map(result_dir + "/data/" + file_name);
        std::cout << "Loaded SLAM poses" << std::endl;

        associate(poses_gt_p, poses_result_p, FLAGS_offset, FLAGS_maxdiff);

        // plot status
        mail->msg("Processing: %s, poses: %d/%d",file_name, poses_result_p.size(), poses_gt_p.size());

        // check for errors
        if (poses_gt.empty() || poses_result.size()!=poses_gt.size()) {
            mail->msg("ERROR: Couldn't read (all) poses of: %s", file_name);
            return false;
        }

        // compute sequence errors
//        vector<errors> seq_err = trajectoryDistances_t(pose_pairs);
        vector<errors> seq_err = calcSequenceErrors(poses_gt,poses_result);
//        vector<errors> seq_err = calSequenceError_t(pose_pairs);
        saveSequenceErrors(seq_err,error_dir + "/" + file_name);

        // add to total errors
        total_err.insert(total_err.end(),seq_err.begin(),seq_err.end());

        // save + plot bird's eye view trajectories
        savePathPlot(poses_gt,poses_result,plot_path_dir + "/" + file_name);
        vector<int32_t> roi = computeRoi(poses_gt,poses_result);
        plotPathPlot(plot_path_dir,roi,i);

        // save + plot individual errors
        char prefix[16];
        sprintf(prefix,"%02d",i);
        saveErrorPlots(seq_err,plot_error_dir,prefix);
        plotErrorPlots(plot_error_dir,prefix);
    }

    // save + plot total errors + summary statistics
    if (total_err.size()>0) {
        char prefix[16];
        sprintf(prefix,"avg");
        saveErrorPlots(total_err,plot_error_dir,prefix);
        plotErrorPlots(plot_error_dir,prefix);
        saveStats(total_err,result_dir);
    }

    // success
    return true;
}

int32_t main (int32_t argc,char *argv[]) {

    gflags::ParseCommandLineFlags(&argc, &argv, true);

    cout << "Usage: ./eval_odometry [optional: -num=1 -prefix=try1 -offset=0 -maxdiff=0.02]" << endl;

    Mail *mail;
    mail = new Mail();
    mail->msg("Customized visual odometry evaluation.");

    // run evaluation
    eval(mail);

    delete mail;
    gflags::ShutDownCommandLineFlags();
    return 0;
}
