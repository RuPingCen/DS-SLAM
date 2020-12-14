#ORB for TUM rgb datasets

datapath="/media/crp/Samsun970CRP/dataSet/TUM/RGBD"

sequencename=(
"offset string" 
"rgbd_dataset_freiburg1_desk" 
"rgbd_dataset_freiburg1_floor" 
"rgbd_dataset_freiburg1_room" 
"rgbd_dataset_freiburg1_rpy" 
"rgbd_dataset_freiburg1_xyz" 
"rgbd_dataset_freiburg2_xyz" 
"rgbd_dataset_freiburg2_360_kidnap" 
"rgbd_dataset_freiburg2_desk" 
"rgbd_dataset_freiburg2_large_no_loop" 
"rgbd_dataset_freiburg2_large_with_loop" 
"rgbd_dataset_freiburg2_pioneer_360" 
"rgbd_dataset_freiburg2_pioneer_slam" 
"rgbd_dataset_freiburg2_pioneer_slam2" 
"rgbd_dataset_freiburg2_pioneer_slam3" 
"rgbd_dataset_freiburg3_nostructure_notexture_far" 
"rgbd_dataset_freiburg3_nostructure_notexture_near_withloop" 
"rgbd_dataset_freiburg3_nostructure_texture_far" 
"rgbd_dataset_freiburg3_nostructure_texture_near_withloop" 
"rgbd_dataset_freiburg3_structure_notexture_far" 
"rgbd_dataset_freiburg3_structure_notexture_near" 
"rgbd_dataset_freiburg3_structure_texture_far" 
"rgbd_dataset_freiburg3_structure_texture_near" 
)
resultname=(
"offset string" 
"fr1_desk" 
"fr1_floor" 
"fr1_room" 
"fr1_rpy" 
"fr1_xyz" 
"fr2_xyz" 
"fr2_360_kidnap" 
"fr2_desk" 
"fr2_large_no_loop" 
"fr2_large_with_loop" 
"fr2_pioneer_360" 
"fr2_pioneer_slam" 
"fr2_pioneer_slam2" 
"fr2_pioneer_slam3" 
"fr3_nostructure_notexture_far" 
"fr3_nostructure_notexture_near_withloop" 
"fr3_nostructure_texture_far" 
"fr3_nostructure_texture_near_withloop" 
"fr3_structure_notexture_far" 
"fr3_structure_notexture_near" 
"fr3_structure_texture_far" 
"fr3_structure_texture_near" 
)
#./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml /media/crp/Samsun970CRP/dataSet/TUM/RGBD/rgbd_dataset_freiburg1_room/ /media/crp/Samsun970CRP/dataSet/TUM/RGBD/rgbd_dataset_freiburg1_room/associate.txt 

num=1
while [ $num -le 5 ]
do
	#循环运行10次
	times_k=1
	while [ $times_k -le 10 ]
	do
		path_tem=$datapath/${sequencename[num]}
		echo $path_tem
		./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml $path_tem/ $path_tem/associate.txt
		sleep 1
		mv CameraTrajectory.txt  result/LK_ORBSLAM2_CameraTrajectory_TUM_${resultname[num]}_${times_k}.txt
		mv trackingtime.txt result/LK_ORBSLAM2_trackingtime_TUM_${resultname[num]}_${times_k}.txt

		#evo_ape euroc gt est -va --plot --plot_mode xy --save_results out
		times_k=$(( $times_k + 1 ))
	done
 	num=$(( $num + 1 ))
done

num=6
while [ $num -le 12 ]
do
	#循环运行10次
	times_k=1
	while [ $times_k -le 10 ]
	do
		path_tem=$datapath/${sequencename[num]}
		echo $path_tem
		./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM2.yaml $path_tem/ $path_tem/associate.txt
		sleep 1
		mv CameraTrajectory.txt  result/LK_ORBSLAM2_CameraTrajectory_TUM_${resultname[num]}_${times_k}.txt
		mv trackingtime.txt result/LK_ORBSLAM2_trackingtime_TUM_${resultname[num]}_${times_k}.txt

		#evo_ape euroc gt est -va --plot --plot_mode xy --save_results out
		times_k=$(( $times_k + 1 ))
	done
 	num=$(( $num + 1 ))
done
