import pickle

def load_info(path):
    '''
    Read a dictionary from a pickle file including the information of dataset
    :param path: path to the pickle file
    :return: loaded info
    '''
    with open(path, 'rb') as f:
        return pickle.load(f)

def cut_info(info,size):
	#only take desired number of input files for training
	return {'src':info['src'][0:size],'tgt':info['tgt'][0:size],'rot':info['rot'][0:size],'trans':info['trans'][0:size],'overlap':info['overlap'][0:size]}

def save_plk(info):
	#same info as plk_file
	with open('small_train_info.pkl','wb') as f:
		pickle.dump(info,f)

testinfo=load_info("C:\\Users\\niede\\Documents\\Studium\\Mathematics_in_SE\\SS22\\Case_Studies\\RIGA\\configs\\tdmatch\\train_info.pkl")
new=cut_info(testinfo,10)
save_plk(new)