#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <limits>
#include <fstream>

#include "mail.h"
#include "matrix.h"

using namespace std;

// static parameter
// float lengths[] = {5,10,50,100,150,200,250,300,350,400};
float lengths[] = {100,200,300,400,500,600,700,800};
int32_t num_lengths = 8;

struct errors {
    int32_t first_frame;
    float   r_err;
    float   t_err;
    float   len;
    float   speed;
    errors (int32_t first_frame,float r_err,float t_err,float len,float speed) :
            first_frame(first_frame),r_err(r_err),t_err(t_err),len(len),speed(speed) {}
};

// Load poses from file into a vector
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

// Compute distances from the starting pose
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

// Finds a frame that is 
int32_t lastFrameFromSegmentLength(vector<float> &dist,int32_t first_frame,float len, vector<pair<bool, int> > &poses_saved) {
    for (int32_t i=first_frame; i<dist.size(); i++)
        // If the pose is far away enough
        if (dist[i]>(dist[first_frame]+len))
            // If the pose is saved, return the index in the stored poses
            if (poses_saved[i].first){
                return poses_saved[i].second;
            }
    // return i;
    // If cannot find a pose that far enough away
    return -1;
}

// Computes the rotation error
inline float rotationError(Matrix &pose_error) {
    float a = pose_error.val[0][0];
    float b = pose_error.val[1][1];
    float c = pose_error.val[2][2];
    float d = 0.5*(a+b+c-1.0);
    return acos(max(min(d,1.0f),-1.0f));
}

// Computes the translation error
inline float translationError(Matrix &pose_error) {
    float dx = pose_error.val[0][3];
    float dy = pose_error.val[1][3];
    float dz = pose_error.val[2][3];
    return sqrt(dx*dx+dy*dy+dz*dz);
}

// Computes the 
vector<errors> calcSequenceErrors (vector<Matrix> &poses_gt, vector<Matrix> &poses_result, vector<pair<bool, int> > &poses_saved) {

    // error vector
    vector<errors> err;

    // parameters
    int32_t step_size = 10; // every second

    // pre-compute distances (from ground truth as reference) from the starting pose
    // Assuming that the error isn't that much
    vector<float> dist = trajectoryDistances(poses_gt);

    // for all start positions find
    for (int32_t first_frame=0; first_frame<poses_gt.size(); first_frame+=step_size) {
        // Make sure that have a stored pose for the first frame
        // Case if not stored / doesn't exist
        while (first_frame < poses_result.size() && !poses_saved[first_frame].first)
        {
            first_frame++;
        }

        // Exit if there are no more poses
        if (!(first_frame < poses_result.size()))
        {
            std::cout << "NO MORE POSES!" << std::endl;
            break;
        }

        // for all segment lengths compute the frame for the other end
        for (int32_t i=0; i<num_lengths; i++) {

            // current length
            float len = lengths[i];

            // compute last frame
            int32_t last_frame = lastFrameFromSegmentLength(dist,first_frame,len, poses_saved);

            // continue, if sequence not long enough
            // TODO: should be able to break out of the loop b/c all the longer lengths will be too long?
            if (last_frame==-1)
                break;
            // continue;

            // compute rotational and translational errors
            Matrix pose_delta_gt     = Matrix::inv(poses_gt[first_frame])*poses_gt[last_frame];
            Matrix pose_delta_result = Matrix::inv(poses_result[first_frame])*poses_result[last_frame];
            Matrix pose_error        = Matrix::inv(pose_delta_result)*pose_delta_gt;
            float r_err = rotationError(pose_error);
            float t_err = translationError(pose_error);

            // compute speed
            float num_frames = (float)(last_frame-first_frame+1);
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
    int32_t step_size = 1;

    // open file
    FILE *fp = fopen(file_name.c_str(),"w");

    // save x/z coordinates of all frames to file
    for (int32_t i=0; i<poses_result.size(); i+=step_size)
    {
        fprintf(fp,"%f %f %f %f\n",poses_gt[i].val[0][3],poses_gt[i].val[2][3],
                poses_result[i].val[0][3],poses_result[i].val[2][3]);
    }

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

// Load the saved_poses file
std::vector<std::pair<bool, int> > load_saved_poses(std::string saved_poses_path)
{
    std::vector<std::pair<bool, int> > saved_poses;

    ifstream file (saved_poses_path.c_str());
    std::pair<bool, int>  value;

    int num_vals = 0;
    file >> num_vals;

    std::cout << "Num vals is: " << num_vals << std::endl;

    for (int i = 0; i < num_vals; i++)
    {
        int first_val;
        int second_val;
        file >> first_val;
        file >> second_val;
        value.first = first_val;
        value.second = second_val;
        saved_poses.push_back(value);
    }

    return saved_poses;
}

void saveLengthErrorPlots(vector<errors> &seq_err,string plot_error_dir,char* prefix) {

    // for each segment length do
    for (int32_t i=0; i<num_lengths; i++) {

        // current length
        int len = lengths[i];

        // file names
        char file_name_tl[1024]; sprintf(file_name_tl,"%s/%s_%02d_tl.txt",plot_error_dir.c_str(),prefix, len);
        char file_name_rl[1024]; sprintf(file_name_rl,"%s/%s_%02d_rl.txt",plot_error_dir.c_str(),prefix, len);

        // open files
        FILE *fp_tl = fopen(file_name_tl,"w");
        FILE *fp_rl = fopen(file_name_rl,"w");

        std::cout << "file_name_tl: " << file_name_tl << std::endl;
        std::cout << "file_name_rl: " << file_name_rl << std::endl;

        // for all errors get the lengths that are of the current length and save the errors
        for (vector<errors>::iterator it=seq_err.begin(); it!=seq_err.end(); it++) {
            if (fabs(it->len-len)<1.0) {
                fprintf(fp_tl,"%d %f\n",it->first_frame,it->t_err);
                fprintf(fp_rl,"%d %f\n",it->first_frame,it->r_err);
            }
        }

        // close files
        fclose(fp_tl);
        fclose(fp_rl);
    }
}

void plotLengthErrorPlots (string dir,char* prefix) {

    char command[1024];

    // for each segment length do
    for (int32_t k=0; k<num_lengths; k++) {
        // current length
        int len = lengths[k];
        // for both error plots do
        for (int32_t i=0; i<2; i++) {

            // create suffix
            char suffix[16];
            switch (i) {
                case 0: sprintf(suffix,"tl"); break;
                case 1: sprintf(suffix,"rl"); break;
            }

            // gnuplot file name
            char file_name[1024]; char full_name[1024];
            sprintf(file_name,"%s_%02d_%s.gp",prefix,len,suffix);
            sprintf(full_name,"%s/%s",dir.c_str(), file_name);

            // create png + eps
            for (int32_t j=0; j<2; j++) {

                // open file
                FILE *fp = fopen(full_name,"w");

                // std::cout << "full_name: " << full_name << std::endl;

                // save gnuplot instructions
                if (j==0) {
                    fprintf(fp,"set term png size 500,250 font \"Helvetica\" 11\n");
                    fprintf(fp,"set output \"%s_%02d_%s.png\"\n",prefix,len,suffix);
                } else {
                    fprintf(fp,"set term postscript eps enhanced color\n");
                    fprintf(fp,"set output \"%s_%02d_%s.eps\"\n",prefix,len,suffix);
                }

                // start plot at 0
                fprintf(fp,"set size ratio 0.5\n");
                fprintf(fp,"set yrange [0:*]\n");

                // x label
                if (i<=1) fprintf(fp,"set xlabel \"Frame Number\"\n");
                else      fprintf(fp,"set xlabel \"Speed [km/h]\"\n");

                // y label
                if (i==0 || i==2) fprintf(fp,"set ylabel \"Translation Error [%%]\"\n");
                else              fprintf(fp,"set ylabel \"Rotation Error [deg/m]\"\n");

                // plot error curve
                fprintf(fp,"plot \"%s_%02d_%s.txt\" using ",prefix,len,suffix);
                switch (i) {
                    case 0: fprintf(fp,"1:($2*100) title 'Translation Error with Length_%02d'", len); break;
                    case 1: fprintf(fp,"1:($2*57.3) title 'Rotation Error with Length_%02d'", len); break;
                }
                fprintf(fp," lc rgb \"#0000FF\" pt 4 w linespoints\n");

                // close file
                fclose(fp);

                // run gnuplot => create png + eps
                sprintf(command,"cd %s; gnuplot %s",dir.c_str(),file_name);
                system(command);
            }

            // create pdf and crop
            sprintf(command,"cd %s; ps2pdf %s_%02d_%s.eps %s_%02d_%s_large.pdf",dir.c_str(),prefix,len,suffix,prefix,len,suffix);
            system(command);
            sprintf(command,"cd %s; pdfcrop %s_%02d_%s_large.pdf %s_%02d_%s.pdf",dir.c_str(),prefix,len,suffix,prefix,len,suffix);
            system(command);
            sprintf(command,"cd %s; rm %s_%02d_%s_large.pdf",dir.c_str(),prefix,len,suffix);
            system(command);
        }
    }
}

// Computes the rotation error
inline float indiv_trans_rmse(Matrix &pose1, Matrix &pose2) {
    float dx = pose1.val[0][3] - pose2.val[0][3];
    float dy = pose1.val[1][3] - pose2.val[1][3];
    float dz = pose1.val[2][3] - pose2.val[2][3];
    return sqrt(dx*dx+dy*dy+dz*dz);
}

// Computes the translational RMSE
vector<std::pair<int, float> > compute_trans_rmse (vector<Matrix> &poses_gt, vector<Matrix> &poses_result, vector<pair<bool, int> > &poses_saved) {

    // error vector
    vector<std::pair<int, float> > trans_rmse;

    for (int32_t frame_idx=0; frame_idx < poses_gt.size(); frame_idx+= 1) {
        // Make sure that have a stored pose for the first frame
        // Case if not stored / doesn't exist
        while (frame_idx < poses_result.size() && !poses_saved[frame_idx].first)
        {
            frame_idx++;
        }

        // Exit if there are no more poses
        if (!(frame_idx < poses_result.size()))
        {
            std::cout << "NO MORE POSES!" << std::endl;
            break;
        }

        float indiv_rmse = indiv_trans_rmse(poses_result[poses_saved[frame_idx].second], poses_gt[frame_idx]);

        trans_rmse.emplace_back(frame_idx, indiv_rmse);
    }

    // return error vector
    return trans_rmse;
}

void save_trans_rmse (std::vector<std::pair<int, float> > trans_rmse, string file_name) {

    // open file
    FILE *fp;
    fp = fopen(file_name.c_str(),"w");

    for (int i = 0; i < trans_rmse.size(); i++)
    {
        fprintf(fp,"%d %f\n",trans_rmse[i].first,trans_rmse[i].second);
    }

    // close file
    fclose(fp);
}

void plot_trans_rmse (string dir,char* prefix) {

// } (std::vector<std::pair<int, float> > trans_rmse,string dir, string file_name) {

    char command[1024];

    // create suffix
    char suffix[16];
    sprintf(suffix,"rmse");

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
        fprintf(fp,"set xlabel \"Frame Number\"\n");

        // y label
        fprintf(fp,"set ylabel \"Translation Error [m]\"\n");

        // plot error curve
        fprintf(fp,"plot \"%s_%s.txt\" using ",prefix,suffix);
        fprintf(fp,"($1):($2) title 'Translation RMSE'");
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


    // // create png + eps
    // for (int32_t j=0; j<2; j++) {

    //   // open file
    //   FILE *fp = fopen(file_name.c_str(),"w");

    //   // save gnuplot instructions
    //   if (j==0) {
    //     fprintf(fp,"set term png size 500,250 font \"Helvetica\" 11\n");
    //     fprintf(fp,"set output \"%s.png\"\n","rmse_png");
    //   } else {
    //     fprintf(fp,"set term postscript eps enhanced color\n");
    //     fprintf(fp,"set output \"%s.eps\"\n","rmse_eps");
    //   }

    //   // start plot at 0
    //   fprintf(fp,"set size ratio 0.5\n");
    //   fprintf(fp,"set yrange [0:*]\n");

    //   // x label
    //   fprintf(fp,"set xlabel \"Frame Number\"\n");

    //   // y label
    //   fprintf(fp,"set ylabel \"Translation Error [%%]\"\n");

    //   // plot error curve
    //   fprintf(fp,"plot \"rmse.txt\" using ");
    //   fprintf(fp,"($1*3.6):($2*100) title 'Translation Error'");
    //   fprintf(fp," lc rgb \"#0000FF\" pt 4 w linespoints\n");

    //   // close file
    //   fclose(fp);

    //   // run gnuplot => create png + eps
    //   sprintf(command,"cd %s; gnuplot %s",dir.c_str(),file_name.c_str());
    //   system(command);
    // }

    // // // create pdf and crop
    // // sprintf(command,"cd %s; ps2pdf %s_%s.eps %s_%s_large.pdf",dir.c_str(),prefix,suffix,prefix,suffix);
    // // system(command);
    // // sprintf(command,"cd %s; pdfcrop %s_%s_large.pdf %s_%s.pdf",dir.c_str(),prefix,suffix,prefix,suffix);
    // // system(command);
    // // sprintf(command,"cd %s; rm %s_%s_large.pdf",dir.c_str(),prefix,suffix);
    // // system(command);
}

int32_t main (int32_t argc,char *argv[]) {

    // Need 5 arguments
    if (argc!=5) {
        cout << std::endl << "Modified KITTI Odometry Evaluation Code Usage: " << std::endl << std::endl;
        std::cout << "./eval_odometry gnd_truth_path slam_results_path saved_slam_poses_path eval_results_dir" << endl << std::endl;
        std::cout << "where gnd_truth_path is the path to the ground truth txt file from KITTI," << std::endl;
        std::cout << "slam_results_path is the path to the trajectory generated by the SLAM algorithm," << std::endl;
        std::cout << "saved_slam_poses_path is the path to the file specifying which gnd truth poses to compare to and the respective idx in slam_results_path," << std::endl;
        std::cout << "and eval_results_dir is the path to the directory to store the evaluation results." << std::endl;
        return 1;
    }

    // This is arbitrary
    int i = 6;

    // file name
    char file_name[256];
    sprintf(file_name,"%02d.txt",i);

    // read arguments
    std::string gnd_truth_path = argv[1];
    std::string slam_results_path = argv[2];
    std::string saved_poses_path = argv[3];
    std::string eval_results_dir = argv[4];

    string error_path     = eval_results_dir + "/errors.txt";
    string rmse_path     = eval_results_dir + "/rmse_6_rmse.txt";
    string plot_traj_dir  = eval_results_dir + "/traj_plots";
    string plot_error_dir = eval_results_dir + "/error_plots";

    // create output directories
    system(("mkdir " + plot_traj_dir).c_str());
    system(("mkdir " + plot_error_dir).c_str());

    // Load the saved poses
    std::vector<std::pair<bool, int> > poses_saved = load_saved_poses(saved_poses_path);
    std::cout << "Loading saved poses: " << std::endl;

    for (int j = 0; j < poses_saved.size(); j++)
    {
        std::pair<bool, int> indiv_pair = poses_saved[j];
        std::cout << "Pair " << j << ": " << indiv_pair.first << " " << indiv_pair.second << std::endl;
    }

    std::cout << "Loaded saved poses" << std::endl;

    vector<Matrix> poses_gt = loadPoses(gnd_truth_path);
    std::cout << "Loaded gt result poses" << std::endl;

    vector<Matrix> poses_result = loadPoses(slam_results_path);
    std::cout << "Loaded SLAM poses" << std::endl;

    // compute sequence errors
    vector<errors> seq_err = calcSequenceErrors(poses_gt, poses_result, poses_saved);
    std::cout << "Done running calcSequenceErrors" << std::endl;

    saveSequenceErrors(seq_err, error_path);
    std::cout << "Saved sequence errors" << std::endl;

    // save + plot bird's eye view trajectories
    savePathPlot(poses_gt,poses_result, plot_traj_dir + "/" + file_name);
    std::cout << "Saved path plot" << std::endl;

    vector<int32_t> roi = computeRoi(poses_gt,poses_result);
    std::cout << "Finished computeRoi" << std::endl;

    plotPathPlot(plot_traj_dir,roi,i);
    std::cout << "Finished path plot" << std::endl;

    // save + plot individual errors
    char prefix[16];
    sprintf(prefix,"%02d",i);
    saveErrorPlots(seq_err,plot_error_dir,prefix);
    std::cout << "Finished saveErrorPlots" << std::endl;

    plotErrorPlots(plot_error_dir,prefix);
    std::cout << "Finished plotErrorPlots" << std::endl;

    // save + plot errors for each length over trajectory
    char prefix2[16];
    sprintf(prefix2,"%02d_length",i);
    saveLengthErrorPlots(seq_err,plot_error_dir,prefix2);
    std::cout << "Finished saveLengthErrorPlots" << std::endl;

    plotLengthErrorPlots(plot_error_dir,prefix2);
    std::cout << "Finished plotLengthErrorPlots" << std::endl;

    // Computing translational error frame by frame
    std::vector<std::pair<int, float> > trans_rmse = compute_trans_rmse(poses_gt, poses_result, poses_saved);
    std::cout << "Done computing translational RMSE" << std::endl;

    char prefix3[16];
    string rmse = "rmse";
    sprintf(prefix3,"rmse_%d",i);

    save_trans_rmse(trans_rmse, rmse_path);
    std::cout << "Finished saving translational RMSE" << std::endl;

    plot_trans_rmse(eval_results_dir, prefix3);
    std::cout << "Finished saving translational RMSE plots" << std::endl;

    return 0;
}