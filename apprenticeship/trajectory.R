library('dtw')

n_dim_state = 7
n_dim_action = 4
n_dim_kalman_state = n_dim_state + n_dim_action
n_dim_kalman_observed = n_dim_kalman_state

folder = "trajectory_circle"

### Get observed trajectories from file
observed_trajectories = list.files(folder)
observed_trajectories = lapply(observed_trajectories, function(x) {
	paste(folder, "/", x, sep="")
})
observed_trajectories = lapply(observed_trajectories, function(x) {
  as.matrix(read.table(x))
})

### Compute other variables required
num_observed_trajectories = length(observed_trajectories)
observed_time_length = sapply(observed_trajectories, nrow)

hidden_time_length = floor(2.0 * mean(observed_time_length))
hidden_trajectory = matrix(0, hidden_time_length, n_dim_kalman_state)

length(observed_trajectories)
sapply(observed_trajectories, nrow)
hidden_time_length

#### Remove actions
observed_trajectories = lapply(observed_trajectories, function(x) {
  x[, 1:n_dim_state]
})
hidden_trajectory = hidden_trajectory[, 1:n_dim_state]

get_time_warped_series = function (series1, series2) {
  d = dtw(series1, series2, window.type="slantedband", window.size=2)
  p1 = d$index1
  p2 = d$index2
  new_mapping = matrix(0, nrow(series2), ncol(series2))
  for (i in 1:nrow(series2)) {
    new_mapping[i, ] = colMeans(series1[p1[p2 == i], , drop=FALSE])
  }
  #print(p1)
  #print(p2)
  # dtwPlot(d)
  return(new_mapping)
}
#### Run iterations
n = 50
# hidden_trajectory = observed_trajectories[[1]]
while(n) {
  n = n - 1
  warped_obs = lapply(observed_trajectories, function(obs) {
    get_time_warped_series(obs, hidden_trajectory)
  })
  new_hidden_trajectory = Reduce("+", warped_obs) / length(warped_obs)
  if ( abs(sum(hidden_trajectory - new_hidden_trajectory)) < 0.0001 ) {
    break
  }
  hidden_trajectory = new_hidden_trajectory
}
n
hidden_trajectory
write.table(hidden_trajectory, file=paste(folder, "out", sep="_"), row.names=FALSE, col.names=FALSE)
