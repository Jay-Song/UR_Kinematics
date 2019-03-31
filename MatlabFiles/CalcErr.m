function err = CalcErr( target_rot, target_pos, curr_rot, curr_pos )

perr = target_pos - curr_pos;
Rerr = curr_rot'*target_rot; % curr_rot^-1 * tar_rot;
werr = curr_rot* rot2omega(Rerr);

err = [perr; werr];

end

