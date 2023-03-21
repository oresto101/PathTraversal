import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.time.Duration;
import java.time.LocalDateTime;
import java.time.LocalTime;

@Data
@NoArgsConstructor
@AllArgsConstructor
public class Edge {
    String from;
    String to;
    String usedLine;
    LocalDateTime departureTime;
    LocalDateTime arrivalTime;
    Double startStopLat;
    Double startStopLon;
    Double endStopLat;
    Double endStopLon;

    public long getDistanceByTime(){
        return Duration.between(departureTime, arrivalTime).toMinutes();
    }
}