import pickle
import numpy as np
import torch

from glob import glob # to conveniently list files with specific ext.
import os 
import re # to conveniently get substring from a full path by regular expression matching
from tqdm.autonotebook import tqdm


npzs_dir = "../split" # just for Li's machine, where the npzs are located. the next dirs should be 
                      # train/, val/, /4DLoMatch, /4DLoMatch

pth_dir_train = "./overfit" # name this output folder to what you want
info_dir = "./4dmatch" # s.a.a.


def process_npz_item(npz_path:str, save_path:str, pattern:str) -> dict :
    # get the name of the npz scene and create file names for source and target pc in this scene
    prog = re.compile(pattern) #
    fname = prog.findall(npz_path)[0]
    new_src_name, new_tgt_name = fname+'_src.pth', fname+'_tgt.pth'
    
    # read the npz data 
    data = np.load(npz_path)

    # save point clouds
    npz_path_dir = re.findall(r'.+\/', fname)[0] # this pattern extracts 'a/b/' from '/a/b/c'
    
    # make dir where we want to save point cloud if necessary
    save_dir = os.path.join(save_path, npz_path_dir)
    if not os.path.isdir(save_dir):
        os.makedirs(save_dir)
        
    # actually save the point cloud
    torch.save(data['s_pc'], os.path.join(save_path, new_src_name))
    torch.save(data['t_pc'], os.path.join(save_path, new_tgt_name))
    
    # build info pickle
    overlap = data['s_overlap_rate'].item()
    
    return dict(src=new_src_name,
                tgt=new_tgt_name,
                rot=data['rot'],
                trans=data['trans'],
                overlap=overlap)     


def extract_pc_and_pickle_info(key:str, num_sample_npzs:int = 50) -> None:
    pkl_dict = dict(src=[], tgt=[], rot=[], trans=[], overlap=[]) # build a placeholder dict.
    npzs = glob(os.path.join(npzs_dir, f'{key}/**/*.npz'), recursive=True) # list all npz from the targeted dir
    assert len(npzs) != 0, "Find No npzs :("
    
    # subset
    sel_npzs = np.random.choice(npzs, size=num_sample_npzs, replace=False)
    
    # extract pc
    re_pattern = fr"({key}\/.+)\.npz" # this pattern extracts 'a/b/c/d' from 'a/b/c/d.npz'
    for npz in tqdm(sel_npzs):
        for k, v in process_npz_item(npz, pth_dir_train, re_pattern).items():
            pkl_dict[k].append(v)
    
    # pickle info
    if not os.path.isdir(info_dir):
        os.makedirs(info_dir)
    with open(os.path.join(info_dir, f"{key}_info.pkl"), 'wb') as f:
        pickle.dump(pkl_dict, f)
        

if __name__ == '__main__':
    print("Generating Training Set")
    extract_pc_and_pickle_info('train', 50) # make this smaller if you don't have the 15Gb dataset
    print("Generating Validation Set")
    extract_pc_and_pickle_info('val', 50)